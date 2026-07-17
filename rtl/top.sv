`default_nettype none
// Atari 2600 (MiSTer) - 2600-only core.
// Derived from Atari7800_MiSTer by removing every Atari 7800 (MARIA / POKEY /
// YM2151 / 7800 cartridge / BIOS / XM) path. The TIA clock generator and the
// 2600 address decoder that originally lived inside MARIA are reproduced here
// from the core's own 2600-mode logic (see tia_clocks and the decode below), so
// 2600 behaviour is preserved bit-for-bit while no 7800 code remains.

module Atari2600
(
	input  logic        clk_sys,
	input  logic        reset,
	input  logic        pause,
	output logic  [7:0] RED, GREEN, BLUE,
	output logic        HSync, VSync, HBlank, VBlank, VBlank_orig, ce_pix,
	input  logic        PAL,
	input  logic [1:0]  pal_temp,
	input  logic        show_border,
	input  logic        show_overscan,
	output logic [15:0] AUDIO_R, AUDIO_L,
	input  logic [7:0]  cart_out,
	output logic        cart_read,
	output logic [24:0] cart_addr_out,
	input  logic [31:0] cart_size,
	output logic [7:0]  cart_din,
	input  logic        cart_download,
	input  logic [24:0] ioctl_addr,
	input  logic [7:0]  ioctl_dout,
	input  logic        ioctl_wr,
	input  logic [3:0]  idump,
	output logic [3:0]  i_out,
	input  logic [1:0]  ilatch,
	input  logic        tia_stab,
	output logic        tia_f1, tia_pal, tia_en, tia_hsync,
	input  logic [7:0]  PAin, PBin,
	output logic [7:0]  PAout, PBout,
	output logic        PAread,
	input  logic [4:0]  force_bs,
	input  logic        sc,
	input  logic [1:0]  tape_in,
	input  logic        fix_sc_cs,
	input  logic [10:0] ps2_key,
	input  logic        decomb,
	input  logic [4:0]  mapper,
	input  logic        pal_load,
	input  logic [9:0]  pal_addr,
	input  logic        pal_wr,
	input  logic [7:0]  pal_data,
	input  logic        blend,
	input  logic        arm_enable,
	output logic [3:0]  i_read
);

	logic           RDY, tia_RDY, IRQ_n, NMI_n;
	logic [15:0]    cpu_AB;
	logic           cpu_rwn, cpu_halt_n, cpu_released;
	logic [7:0]     read_DB, write_DB;
	logic [7:0]     tia_DB_out, riot_DB_out, cart_DB_out;
	logic [3:0]     audv0, audv1;
	logic           mclk0, mclk1, tia_clk_x2;
	logic           cs_tia, cs_riot, cs_cart;
	logic [7:0]     open_bus;
	logic [24:0]    cart_2600_addr_out;
	logic [7:0]     cart_2600_DB_out;
	logic           pclk1, pclk0, pclk1_t, pclk0_t;
	logic           cart_ce_2600, read_2600, tia_pix_ce;
	logic [1:0]     pause_clock;
	logic [17:0]    cartram_addr26;
	logic           cartram_wr26, cartram_rd26;
	logic [7:0]     cartram_wrdata26;
	logic [7:0]     cartram_data_bram;
	logic [2:0]     tia_luma;
	logic [3:0]     tia_chroma;
	logic           tia_vblank, tia_vsync, tia_hblank, tia_blank_n, tape_audio;

	// 2600 is always selected in this build.
	assign tia_en      = 1'b1;
	assign NMI_n       = 1'b1;
	assign IRQ_n       = 1'b1;
	assign cpu_halt_n  = 1'b1;
	assign RDY         = tia_RDY;

	// Track the data bus because FPGAs have no internal tri-state logic.
	always_ff @(posedge clk_sys) begin
		pause_clock <= pause ? pause_clock + 1'd1 : {1'b0, mclk1};
		open_bus   <= (~cpu_rwn ? write_DB : read_DB);
	end

	// 2600 memory map decode (TIA / RIOT / CART chip selects):
	//   TIA   : A12 == 0 && A7 == 0
	//   RIOT  : A12 == 0 && A7 == 1
	//   CART  : A12 == 1   (covers $1000-$FFFF, incl. bankswitched images)
	assign cs_tia  = ~cpu_AB[12] & ~cpu_AB[7];
	assign cs_riot = ~cpu_AB[12] &  cpu_AB[7];
	assign cs_cart =  cpu_AB[12];
	assign PAread  = cs_riot && ~|cpu_AB[4:0] && cpu_rwn && pclk0;

	always_comb begin
		read_DB = open_bus;
		if (cs_tia)  read_DB = {tia_DB_out[7:6], open_bus[5:0]};
		if (cs_riot) read_DB = riot_DB_out;
		if (cs_cart) read_DB = cart_DB_out;
		pclk0 = pclk0_t;
		pclk1 = pclk1_t;
	end

	// TIA 2x colour-clock enable (tia_clk_x2) and master clocks mclk0/mclk1,
	// derived straight from clk_sys. tia_clk_x2 is held off for a few system
	// cycles after reset (start-up delay) before the TIA is enabled.
	tia_clocks clocks
	(
		.clk_sys   (clk_sys),
		.reset     (reset),
		.ce        (~pause),
		.mclk0     (mclk0),
		.mclk1     (mclk1),
		.tia_clk_x2(tia_clk_x2)
	);

	TIA tia_inst
	(
		.clk         (clk_sys),
		.ce          (tia_clk_x2),
		.is_7800     (1'b0),
		.phi0        (pclk0_t),
		.phi1        (pclk1_t),
		.phi2        (pclk0),
		.RW_n        (cpu_rwn),
		.rdy         (tia_RDY),
		.addr        ({cpu_AB[5], cpu_AB[4:0]}),
		.d_in        (write_DB),
		.d_out       (tia_DB_out),
		.i           (idump),
		.i_out       (i_out),
		.i4          (ilatch[0]),
		.i5          (ilatch[1]),
		.aud0        (audv0),
		.aud1        (audv1),
		.col         (tia_chroma),
		.lum         (tia_luma),
		.BLK_n       (tia_blank_n),
		.sync        (),
		.cs0_n       (~cs_tia),
		.cs2_n       (~cs_tia),
		.rst         (reset),
		.video_ce    (tia_pix_ce),
		.vblank      (tia_vblank),
		.hblank      (),
		.hgap        (tia_hblank),
		.vsync       (tia_vsync),
		.hsync       (tia_hsync),
		.phi1_in     (pclk1),
		.open_bus    (open_bus),
		.decomb      (decomb),
		.cart_ce     (cart_ce_2600),
		.is_pal      (tia_pal),
		.is_f1       (tia_f1),
		.stabilize   (tia_stab),
		.paddle_read (i_read)
	);

	video_mux mux
	(
		.clk_sys        (clk_sys),
		.tia_luma       (tia_luma),
		.tia_chroma     (tia_chroma),
		.tia_hblank     (tia_hblank),
		.tia_vblank     (tia_vblank),
		.tia_hsync      (tia_hsync),
		.tia_vsync      (tia_vsync),
		.tia_pix_ce     (tia_pix_ce),
		.is_PAL         (PAL),
		.pal_temp       (pal_temp),
		.pal_load       (pal_load),
		.pal_data       (pal_data),
		.pal_addr       (pal_addr),
		.pal_wr         (pal_wr),
		.blend          (blend),
		.hblank         (HBlank),
		.vblank         (VBlank),
		.hsync          (HSync),
		.vsync          (VSync),
		.red            (RED),
		.green          (GREEN),
		.blue           (BLUE),
		.pix_ce         (ce_pix)
	);

	// Mono TIA audio (no stereo / multichannel per spec). The non-linear mix
	// table follows https://atariage.com/forums/topic/271920-tia-sound-abnormalities/
	logic [15:0] audio_lut[32];
	assign audio_lut = '{
		16'h0000, 16'h0842, 16'h0FFF, 16'h1745, 16'h1E1D, 16'h2492, 16'h2AAA, 16'h306E,
		16'h35E4, 16'h3B13, 16'h3FFF, 16'h44AE, 16'h4924, 16'h4D64, 16'h5173, 16'h5554,
		16'h590A, 16'h5C97, 16'h5FFF, 16'h6343, 16'h6665, 16'h6968, 16'h6C4D, 16'h6F17,
		16'h71C6, 16'h745C, 16'h76DA, 16'h7942, 16'h7B95, 16'h7DD3, 16'h7FFF, 16'hFFFF
	};
	wire [5:0] aud_index = audv0 + audv1;
	wire [16:0] audio_mix = audio_lut[aud_index] + {tape_audio, 12'd0};
	assign AUDIO_R = audio_mix[15:0];
	assign AUDIO_L = audio_mix[15:0];

	// RIOT (6532) with its internal 128 bytes of RAM - this is the 2600 RAM.
	M6532 riot_inst
	(
		.clk    (clk_sys),
		.ce     (pclk0),
		.res_n  (~reset),
		.addr   (cpu_AB[6:0]),
		.RW_n   (cpu_rwn),
		.d_in   (write_DB),
		.d_out  (riot_DB_out),
		.RS_n   (cpu_AB[9]),
		.IRQ_n  (),
		.CS1    (cpu_AB[7]),
		.CS2_n  (~cs_riot),
		.PA_in  (PAin),
		.PA_out (PAout),
		.PB_in  (PBin),
		.PB_out (PBout)
	);

	M6502C cpu_inst
	(
		.pclk1      (pclk1),
		.clk_sys    (clk_sys),
		.reset      (reset),
		.AB         (cpu_AB),
		.DB_IN      (read_DB),
		.DB_OUT     (write_DB),
		.RD         (cpu_rwn),
		.IRQ_n      (IRQ_n),
		.NMI_n      (NMI_n),
		.RDY        (RDY),
		.halt_n     (cpu_halt_n),
		.is_halted  (cpu_released)
	);

	assign VBlank_orig = tia_vblank;

	assign cart_2600_addr_out[24:19] = '0;
	assign cart_din = cpu_rwn ? read_DB : write_DB;

	cart2600 cart2600
	(
		.d_out         (cart_2600_DB_out),
		.d_in          (cart_din),
		.a_in          (cpu_AB[12:0]),
		.clk           (clk_sys),
		.reset         (reset),
		.ce            (cart_ce_2600),
		.phi1          (pclk1),
		.oe            (),
		.open_bus      (open_bus),
		.sc            (sc),
		.mapper        (|mapper ? mapper : force_bs),
		.arm_enable    (arm_enable),
		.cart_download (cart_download),
		.ioctl_addr    (ioctl_addr[18:0]),
		.ioctl_dout    (ioctl_dout),
		.ioctl_wr      (ioctl_wr),
		.rom_do        (cart_out),
		.rom_size      (cart_size[18:0]),
		.rom_a         (cart_2600_addr_out[18:0]),
		.rom_read      (read_2600),
		.cartram_addr  (cartram_addr26),
		.cartram_wr    (cartram_wr26),
		.cartram_rd    (cartram_rd26),
		.cartram_wrdata(cartram_wrdata26),
		.cartram_data  (cartram_data_bram),
		.tape_audio    (tape_audio),
		.tape_in       (tape_in),
		.fix_sc_cs     (fix_sc_cs)
	);

	assign cart_addr_out = cart_2600_addr_out;
	assign cart_DB_out   = cart_2600_DB_out;
	assign cart_read     = pause ? ~|pause_clock : read_2600;

	logic [16:0] reset_addr;
	always @(posedge clk_sys) begin :reset_cart
		logic old_reset;
		old_reset <= reset;
		reset_addr <= (reset && ~old_reset) ? 16'd0 : reset_addr + 1'd1;
	end

	// Cartridge RAM (Supercharger / 032 / etc.) lives on-chip.
	spram #(.addr_width(17), .mem_name("CART")) cart_ram
	(
		.clock   (clk_sys),
		.address (reset ? reset_addr : cartram_addr26),
		.data    (reset ? 8'd0 : cartram_wrdata26),
		.wren    (reset ? 1'd1 : cartram_wr26),
		.q       (cartram_data_bram),
		.cs      (~pause)
	);

endmodule


// TIA clock generator. Produces the TIA 2x colour-clock enable (tia_clk_x2) and
// the master clocks mclk0/mclk1 directly from clk_sys. tia_clk_x2 is gated off
// until a short start-up count elapses, after which the TIA is enabled. There is
// no 7800 / MARIA logic here - this is the standalone 2600 clocking only.
module tia_clocks
(
	input  logic clk_sys, reset, ce,
	output logic mclk0, mclk1, tia_clk_x2
);
	logic        clk_toggle, tia_clk_en;
	logic [3:0]  tia_enable_count;

	assign tia_clk_x2 = tia_clk_en && mclk0;

	always @(posedge clk_sys) begin
		if (reset) begin
			{clk_toggle, mclk0, mclk1} <= 0;
			tia_enable_count <= 2;
		end else if (ce) begin
			if (mclk1 && |tia_enable_count) tia_enable_count <= tia_enable_count - 1'd1;
			{mclk0, mclk1, clk_toggle} <= {clk_toggle, ~clk_toggle, ~clk_toggle};
		end
		tia_clk_en <= ~|tia_enable_count;
	end
endmodule


// 6502 CPU wrapper (T65). The 2600 uses a 6507; T65 implements the 6502 core.
module M6502C
(
	input         pclk1,
	input         clk_sys,
	input         reset,
	input  [7:0]  DB_IN,
	input         IRQ_n,
	input         NMI_n,
	input         RDY,
	input         halt_n,
	output [15:0] AB,
	output [7:0]  DB_OUT,
	output        RD,
	output        is_halted
);

	logic cpu_halt_n = 1;
	logic rdy_delay = 1;

	T65 cpu
	(
		.mode    (0),
		.BCD_en  (1),
		.Res_n   (~reset),
		.Clk     (clk_sys),
		.Enable  (pclk1 && cpu_halt_n),
		.Rdy     (rdy_delay),
		.IRQ_n   (IRQ_n),
		.NMI_n   (NMI_n),
		.R_W_n   (RD),
		.A       (AB),
		.DI      (RD ? DB_IN : DB_OUT),
		.DO      (DB_OUT)
	);

	always @(posedge clk_sys) begin
		is_halted <= ~cpu_halt_n;
		if (reset) begin
			is_halted <= 0;
			cpu_halt_n <= 1;
			rdy_delay <= 1;
		end else if (pclk1) begin
			cpu_halt_n <= halt_n;
			rdy_delay <= RDY;
		end
	end

endmodule
