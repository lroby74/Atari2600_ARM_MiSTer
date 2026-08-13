`default_nettype none
////////////////////////////////////////////////////////////////////////////////
// Atari 2600 (MiSTer) - boot splash background music.
//
// A tiny, self-contained melody player used while the startup splash screen
// is shown. It replaces the TIA audio (which at cold boot is a DC / locked
// tone - the "suono fisso" symptom) with a short looping arpeggio, then goes
// silent the moment the splash is dismissed.
//
// NOTE ON SOURCE: the user referenced the background music of the "old
// standalone Atari2600 core". The available reference (Atari2600_ARM_MiSTer_ref,
// the VHDL A2601 core) contains only DPC *in-game* music (DpcMusicModes), not
// a boot-screen melody; the Atari7800_MiSTer reference has no boot-screen code
// at all (only the 7800title.bin fallback cartridge ROM). Since no code-level
// boot melody exists in the available references, this module implements an
// equivalent self-contained melody. If the exact old-core tune file is
// provided it can be adapted 1:1 by replacing the note periods below.
//
// The output is a 16-bit signed sample stream (matching AUDIO_L / AUDIO_R of
// the emu module). The downstream audio_out.sv sigma-delta DAC samples it, so
// we only need to produce a square wave at the desired note frequency.
//
// Style: SystemVerilog-2005, compact "Sorgelig approved" formatting.
////////////////////////////////////////////////////////////////////////////////
module boot_music
#(
	parameter CLK_RATE = 7159090   // system clock frequency in Hz (~7.16 MHz NTSC)
)
(
	input  logic        clk_sys,
	input  logic        reset,
	input  logic        enable,     // = startup (splash active)
	output logic [15:0] audio_l,
	output logic [15:0] audio_r
);

	// Note periods expressed as clk_sys ticks (CLK_RATE / note_frequency).
	localparam [23:0] P_C5 = 24'd13683;  // 523.25 Hz
	localparam [23:0] P_E5 = 24'd10863;  // 659.25 Hz
	localparam [23:0] P_G5 = 24'd9133;   // 783.99 Hz
	localparam [23:0] P_C6 = 24'd6842;   // 1046.5 Hz
	localparam [23:0] P_A4 = 24'd16275;  // 440.00 Hz
	localparam [23:0] P_G4 = 24'd12182;  // 392.00 Hz

	localparam [3:0] NOTE_COUNT = 4'd12;

	// Looping arpeggio (C5 E5 G5 C6 A4 G5 E5 C5 C5 G5 C6 G5).
	// Implemented as a function so it is portable across Icarus/Verilator
	// (array localparams with '{...} initialisers are not accepted by Icarus).
	function [23:0] note_period;
		input [3:0] idx;
		begin
			case (idx)
				4'd0:  note_period = P_C5;
				4'd1:  note_period = P_E5;
				4'd2:  note_period = P_G5;
				4'd3:  note_period = P_C6;
				4'd4:  note_period = P_A4;
				4'd5:  note_period = P_G5;
				4'd6:  note_period = P_E5;
				4'd7:  note_period = P_C5;
				4'd8:  note_period = P_C5;
				4'd9:  note_period = P_G5;
				4'd10: note_period = P_C6;
				4'd11: note_period = P_G5;
				default: note_period = P_C5;
			endcase
		end
	endfunction

	// ~0.30 s per note @ 7.16 MHz.
	localparam [31:0] NOTE_TICKS = 32'd2147000;

	logic [3:0]  note_idx;
	logic [31:0] note_timer;
	logic [23:0] phase;
	logic        sq;

	always @(posedge clk_sys) begin
		if (reset) begin
			note_idx   <= 0;
			note_timer <= 0;
			phase      <= 0;
			sq         <= 0;
		end else if (enable) begin
			// Toggle the square wave at the current note frequency.
			if (phase >= (note_period(note_idx) >> 1)) begin
				phase <= 0;
				sq    <= ~sq;
			end else begin
				phase <= phase + 1'd1;
			end
			// Advance to the next note after NOTE_TICKS.
			if (note_timer >= NOTE_TICKS) begin
				note_timer <= 0;
				if (note_idx == (NOTE_COUNT-1)) note_idx <= 0;
				else                            note_idx <= note_idx + 1'd1;
			end else begin
				note_timer <= note_timer + 1'd1;
			end
		end else begin
			note_idx   <= 0;
			note_timer <= 0;
			phase      <= 0;
			sq         <= 0;
		end
	end

	// Moderate volume; signed 16-bit square wave (or silence when disabled).
	localparam [15:0] AMP = 16'h3000;
	assign audio_l = enable ? (sq ? AMP : (~AMP + 1'b1)) : 16'sh0000;
	assign audio_r = audio_l;

endmodule
