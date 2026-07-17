// Atari 2600 (MiSTer) - video multiplexer (2600 TIA path only).
// The MARIA (7800) video inputs have been removed; this module now handles
// only the TIA pixel stream and the palette lookup.

module video_mux
(
	input  logic       clk_sys,

	input  logic [2:0] tia_luma,
	input  logic [3:0] tia_chroma,
	input  logic       tia_hblank,
	input  logic       tia_vblank,
	input  logic       tia_hsync,
	input  logic       tia_vsync,
	input  logic       tia_pix_ce,

	input  logic [1:0] pal_temp,
	input  logic       is_PAL,
	input  logic       pal_load,
	input  logic [7:0] pal_data,
	input  logic [9:0] pal_addr,
	input  logic       pal_wr,
	input  logic       blend,

	output logic       hblank,
	output logic       vblank,
	output logic       hsync,
	output logic       vsync,
	output logic [7:0] red,
	output logic [7:0] green,
	output logic [7:0] blue,
	output logic       pix_ce
);

logic [23:0] out_color, nwarm_color, ncool_color, nhot_color,
	pwarm_color, pcool_color, phot_color, custom_color, old_color;

// If luma alternates intensity at 320 pixels per line:
// If colorburst is low, it will manifest as +blue
// If colorburst is high, it will manifest as +yellow
// Luma will become filtered and blend
// Chroma will end up blending more smoothly

wire pix_ce_immediate = tia_pix_ce;
logic pix_ce_delayed;
logic [7:0] yuv_index, old_yuv_index;
logic [7:0][1:0] last_color;
logic [3:0] tia_chroma_region;

// PAL 2600 chroma remap (DRAM refresh causes the chroma to shift by one step).
// Values below map the 2600 chroma nibble to the palette index used here.

wire [3:0] pal_2600_chroma[16] = '{
	4'h0, 4'h0, 4'h2, 4'hD,
	4'h3, 4'hc, 4'h4, 4'hb,
	4'h5, 4'ha, 4'h6, 4'h9,
	4'h7, 4'h8, 4'h0, 4'h0
};

always_comb begin
	tia_chroma_region = is_PAL ? pal_2600_chroma[tia_chroma] : tia_chroma;
	out_color = nwarm_color;

	yuv_index = ~pix_ce_immediate ? {old_yuv_index[7:1], 1'b0} : {tia_chroma_region, {tia_luma, 1'b0}};

	case ({is_PAL, pal_temp})
		0: out_color = nwarm_color;
		1: out_color = ncool_color;
		2: out_color = nhot_color;
		3: out_color = custom_color;
		4: out_color = pwarm_color;
		5: out_color = pcool_color;
		6: out_color = phot_color;
		7: out_color = custom_color;
	 default: ;
	endcase

end


// wire signed [6:0] old_diff = $signed{1'b0, last_color[0][3:0]} - $signed{1'b0, last_color[1][3:0]};
// wire signed [6:0] new_diff = $signed{1'b0, last_color[0][3:0]} - $signed{1'b0, yuv_index[3:0]};
// wire signed [6:0] old_abs = 

logic [15:0] pal_buff;
logic [7:0] pal_mux_addr;
logic [1:0] pal_count = 0;
logic old_vblank;

wire [23:0] blend_color = {
	{1'b0, old_color[23:17]} + out_color[23:17],
	{1'b0, old_color[15:9]} + out_color[15:9],
	{1'b0, old_color[7:1]} + out_color[7:1]
};

always @(posedge clk_sys) begin
	if (pal_load) begin
		if (pal_wr) begin
			pal_count <= pal_count == 2 ? 2'd0 : pal_count + 1'd1;
			case (pal_count)
				0: pal_buff[15:8] <= pal_data;
				1: pal_buff[7:0] <= pal_data;
				2: pal_mux_addr <= pal_mux_addr + 1'd1;
			endcase
		end
	end else begin
		pal_mux_addr <= 0;
		pal_count <= 0;
	end
	if (pix_ce_immediate) begin
		old_color <= out_color;
	end

	pix_ce_delayed <= pix_ce_immediate;
	pix_ce <= pix_ce_delayed;
	if (pix_ce_delayed) begin
		old_vblank <= tia_vblank;
		old_yuv_index <= yuv_index;
		last_color <= {last_color[0], yuv_index};
		{red, green, blue} <= blend ? blend_color : out_color;
		vsync <= tia_vsync;
		vblank <= blend ? (old_vblank | tia_vblank) : tia_vblank;
		hsync <= tia_hsync;
		hblank <= tia_hblank;
	end
end

// Palettes research by Robert Tuccitto represents three different console temperatures.
// Having a range of options for whatever the game developer happened to optimize towards is a
// good idea. According to Robert, the three temperatures represent the following chroma shifts:
// warm is 26.7 degrees, cool is 25.7 degrees, and hot is 27.7 degrees.
// Last updated 3/13/2021

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/NWARM.mif")
) nwarm
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (nwarm_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/NCOOL.mif")
) ncool
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (ncool_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/NHOT.mif")
) nhot
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (nhot_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/PWARM.mif")
) pwarm
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (pwarm_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/PCOOL.mif")
) pcool
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (pcool_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/PHOT.mif")
) phot
(
	.clock   (clk_sys),
	.address (yuv_index),
	.q       (phot_color)
);

spram #(
	.addr_width(8),
	.data_width(24),
	.mem_init_file("rtl/palettes/PHOT.mif")
) custom
(
	.clock   (clk_sys),
	.data    ({pal_buff, pal_data}),
	.wren    (pal_load && pal_wr && (pal_count == 2)),
	.address (pal_load ? pal_mux_addr : yuv_index),
	.q       (custom_color)
);


endmodule

//     red   gree  blue
// 0 | 4'h7, 4'h5, 4'h0
// 1 | 4'h5, 4'h6, 4'h0 // yellow-ish
// 2 | 4'h3, 4'h7, 4'h0 // green-ish
// 3 | 4'h2, 4'h8, 4'h1 // green peak
// 4 | 4'h1, 4'h7, 4'h3
// 5 | 4'h1, 4'h7, 4'h7
// 6 | 4'h2, 4'h6, 4'ha
// 7 | 4'h3, 4'h5, 4'hc 
// 8 | 4'h5, 4'h4, 4'hc // blue peak?
// 9 | 4'h7, 4'h3, 4'hc // magenta-ish
// a | 4'h8, 4'h3, 4'ha 
// b | 4'h9, 4'h3, 4'h7 
// c | 4'h9, 4'h3, 4'h4 // red peak
// d | 4'h8, 4'h4, 4'h1 
// e | 4'h7, 4'h5, 4'h0
// f | 4'h5, 4'h5, 4'h5