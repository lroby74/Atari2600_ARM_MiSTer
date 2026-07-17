`default_nettype none
////////////////////////////////////////////////////////////////////////////////
// Atari 2600 (MiSTer) - startup splash screen overlay (boot screen).
//
// VIDEO-ONLY boot screen. Shows the bitmap text:
//
//        ATARI 2600
//           ARM
//
// rendered with a 5x7 pixel font (bitmaps supplied by the user) and centred
// on BOTH the CRT (15 kHz) and HDMI outputs. It appears exactly ONCE at core
// power-on and is dismissed by the first user input or cartridge load. Audio
// is fully muted at boot (handled in Atari2600.sv: AUDIO_L/R = startup?0:...).
//
// WHY SELF-TIMED SYNC:
//   The module GENERATES its own clean 2600-style synchronisation
//   (HSync/VSync/HBlank/VBlank) while the splash is active. This guarantees
//   the boot screen is displayed even if the TIA is still in reset / has no
//   stable video at cold boot (the "CRT no signal / video lockup" symptom).
//   When inactive it passes the TIA sync and RGB straight through, leaving the
//   surrounding video pipeline (cofi / video_mixer / ascal) untouched.
//
// FONT / RESOURCE STRATEGY ("Sorgelig approved", SystemVerilog-2005 compact):
//   * The 5x7 glyphs are stored as a tiny combinational ROM (one 35-bit
//     constant per character, decoded by a case). No block-RAM, no dividers.
//   * Glyph stride is a POWER OF TWO (8 px = 5 glyph + 3 spacing) so the
//     column index is a pure bit-slice (lx[6:3]) and the in-glyph x index is
//     lx[2:0]; the horizontal decode infers zero logic.
//   * Background is pure black (8'h00) -> compatible with HDMI and CRT 15 kHz
//     and with the downstream pass-through.
//
// Style: SystemVerilog-2005, compact "Sorgelig approved" formatting.
////////////////////////////////////////////////////////////////////////////////
module startup_screen
(
	input  logic        clk_sys,
	input  logic        ce_pix,      // TIA colour-clock enable (228 pulses/line)
	input  logic        HSync, VSync, HBlank, VBlank,  // TIA sync (pass-through)
	input  logic [7:0]  R_in, G_in, B_in,
	input  logic        startup,     // 1 = show splash (self-timed)
	input  logic        is_pal,      // selects vertical centring
	// Synced video output to the pipeline (cofi). Self-generated while active,
	// TIA pass-through while inactive.
	output logic        HSync_out, VSync_out, HBlank_out, VBlank_out,
	output logic [7:0]  R_out, G_out, B_out
);

	// ---- self-timed 2600 geometry (faithful: 228 colour-clocks/line) ------
	localparam [9:0] HTOTAL = 10'd228;
	localparam [9:0] VTOTAL_NTSC = 10'd262;
	localparam [9:0] VTOTAL_PAL  = 10'd312;
	wire  [9:0] vtotal = is_pal ? VTOTAL_PAL : VTOTAL_NTSC;

	// ---- 5x7 bitmap font --------------------------------------------------
	// ch: 0=A 1=T 2=R 3=I 4='2' 5='6' 6='0' 7=M 8=space
	// Each glyph is 7 rows of 5 bits, MSB = leftmost pixel.
	function [4:0] glyph_row;
		input [3:0] ch;
		input [2:0] row;          // 0..6 (top .. bottom)
		reg   [34:0] g;
		reg   [2:0]  r;
		begin
			case (ch)
			4'd0:    g = 35'b01110_10001_10001_11111_10001_10001_10001; // A
			4'd1:    g = 35'b11111_00100_00100_00100_00100_00100_11111; // T
			4'd2:    g = 35'b11110_10001_10001_11110_10100_10010_10001; // R
			4'd3:    g = 35'b11111_00100_00100_00100_00100_00100_11111; // I
			4'd4:    g = 35'b01110_10001_00001_00010_00100_01000_11111; // 2
			4'd5:    g = 35'b01110_10000_10000_11110_10001_10001_01110; // 6
			4'd6:    g = 35'b01110_10001_10011_10101_11001_10001_01110; // 0
			4'd7:    g = 35'b10001_11011_10101_10001_10001_10001_10001; // M
			default: g = 35'b00000_00000_00000_00000_00000_00000_00000; // space
			endcase
			r = (row > 3'd6) ? 3'd6 : row;          // clamp (row is 3 bits)
			glyph_row = g[ (34 - (r*5)) -: 5 ];      // 5-bit slice, row r
		end
	endfunction

	// Line character maps (column -> glyph index).
	function [3:0] top_ch;
		input [3:0] c;   // 0..9  -> "ATARI 2600"
		begin
			case (c)
			4'd0: top_ch = 4'd0; // A
			4'd1: top_ch = 4'd1; // T
			4'd2: top_ch = 4'd0; // A
			4'd3: top_ch = 4'd2; // R
			4'd4: top_ch = 4'd3; // I
			4'd5: top_ch = 4'd8; // space
			4'd6: top_ch = 4'd4; // 2
			4'd7: top_ch = 4'd5; // 6
			4'd8: top_ch = 4'd6; // 0
			4'd9: top_ch = 4'd6; // 0
			default: top_ch = 4'd8;
			endcase
		end
	endfunction

	function [3:0] bot_ch;
		input [3:0] c;   // 0..2  -> "ARM"
		begin
			case (c)
			4'd0: bot_ch = 4'd0; // A
			4'd1: bot_ch = 4'd2; // R
			4'd2: bot_ch = 4'd7; // M
			default: bot_ch = 4'd8;
			endcase
		end
	endfunction

	// ---- free-running 2600-style counters (advanced on ce_pix) ------------
	// Initialised to 0 so simulation does not start in an X state. On real
	// hardware this infers a power-on reset to 0.
	logic [9:0] hcount = 0, vcount = 0;
	always @(posedge clk_sys) begin
		if (ce_pix) begin
			if (hcount == (HTOTAL-1)) begin
				hcount <= 0;
				if (vcount == (vtotal-1)) vcount <= 0;
				else                      vcount <= vcount + 1'd1;
			end else
				hcount <= hcount + 1'd1;
		end
	end

	// ---- self-generated sync (only meaningful while active) ---------------
	// Horizontal: 38-clock blanking, 16-clock HSync pulse.
	// Vertical  : 30-line blanking, 3-line VSync pulse.
	logic hb, vb, hs, vs;
	always @* begin
		hb = (hcount < 10'd38);
		hs = (hcount >= 10'd4) && (hcount < 10'd20);
		vb = (vcount < 10'd30);
		vs = (vcount < 10'd3);
	end

	// ---- text layout (power-of-two stride => bit-slice decode) ------------
	localparam [9:0] X0        = 10'd94;   // text block left edge
	wire  [9:0] V0        = is_pal ? 10'd163 : 10'd138; // vertical centre
	localparam [9:0] TOP_W     = 10'd77;   // "ATARI 2600": 10*8 - 3
	localparam [9:0] BOT_W     = 10'd21;   // "ARM"        : 3*8 - 3
	localparam [9:0] BOT_OFF_X = 10'd28;   // ARM shifted right so it sits
	                                      //   centred under "ATARI 2600"
	localparam [9:0] BOT_OFF_Y = 10'd9;    // 7 glyph rows + 2 blank rows

	// ---- combinational text pixel -----------------------------------------
	// Geometry temporaries are declared at module level and defaulted every
	// cycle, so the block stays purely combinational (no inferred latches).
	logic [9:0] lx;
	logic [3:0] col;
	logic [2:0] xin, rrow;
	logic [4:0] gr;
	logic text_on;
	wire  [9:0] vrow_top = vcount - V0;            // 0..6 inside the top line
	wire  [9:0] vrow_bot = vcount - (V0 + BOT_OFF_Y); // 0..6 inside bottom line
	always @* begin
		text_on = 1'b0;
		lx = 10'd0; col = 4'd0; xin = 3'd0; rrow = 3'd0; gr = 5'd0;
		// --- top line "ATARI 2600" ---
		if ((hcount >= X0) && (hcount < X0 + TOP_W) &&
		    (vcount >= V0) && (vcount < V0 + 10'd7)) begin
			lx   = hcount - X0;
			col  = lx[6:3];     // column = lx / 8  (stride power of two)
			xin  = lx[2:0];     // in-cell x = lx % 8
			rrow = vrow_top[2:0];   // 0..6 (vcount-V0 < 8)
			if (xin < 3'd5) begin
				gr = glyph_row(top_ch(col), rrow);
				text_on = gr[3'd4 - xin];
			end
		end
		// --- bottom line "ARM" (only if top line did not already set) ---
		if (!text_on &&
		    (hcount >= X0 + BOT_OFF_X) && (hcount < X0 + BOT_OFF_X + BOT_W) &&
		    (vcount >= V0 + BOT_OFF_Y) && (vcount < V0 + BOT_OFF_Y + 10'd7)) begin
			lx   = hcount - (X0 + BOT_OFF_X);
			col  = lx[6:3];
			xin  = lx[2:0];
			rrow = vrow_bot[2:0];
			if (xin < 3'd5) begin
				gr = glyph_row(bot_ch(col), rrow);
				text_on = gr[3'd4 - xin];
			end
		end
		if (!startup) text_on = 1'b0;   // gate by startup (clean pass-through)
	end

	localparam [7:0] FG = 8'hFF;   // white text
	localparam [7:0] BG = 8'h00;   // black background

	// ---- output: self-timed while active, TIA pass-through otherwise ------
	always @* begin
		if (startup) begin
			HSync_out  = hs;
			VSync_out  = vs;
			HBlank_out = hb;
			VBlank_out = vb;
			R_out = text_on ? FG : BG;
			G_out = text_on ? FG : BG;
			B_out = text_on ? FG : BG;
		end else begin
			HSync_out  = HSync;
			VSync_out  = VSync;
			HBlank_out = HBlank;
			VBlank_out = VBlank;
			R_out = R_in;
			G_out = G_in;
			B_out = B_in;
		end
	end

endmodule
