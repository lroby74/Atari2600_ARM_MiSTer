// Cyclone-V M10K espliciti per ROM e Display RAM del bridge CDF.
// Stessa regola di harmony_m10k_tdp.sv: TUTTI i parametri _reg_b su CLOCK0.
`default_nettype none

module cdf_rom_m10k #(
  parameter INIT_FILE = ""
)(
  input  wire        clk,
  input  wire [12:0] addr_a,
  input  wire [31:0] data_a,
  input  wire [3:0]  byteena_a,
  input  wire        wren_a,
  output wire [31:0] q_a,
  input  wire [12:0] addr_b,
  output wire [31:0] q_b
);
  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (32), .widthad_a(13), .numwords_a(8192), .width_byteena_a(4),
    .width_b                       (32), .widthad_b(13), .numwords_b(8192), .width_byteena_b(4),
    .outdata_reg_a                 ("CLOCK0"),
    .outdata_reg_b                 ("CLOCK0"),
    .address_reg_b                 ("CLOCK0"),
    .indata_reg_b                  ("CLOCK0"),
    .wrcontrol_wraddress_reg_b     ("CLOCK0"),
    .byteena_reg_b                 ("CLOCK0"),
    .read_during_write_mode_port_a ("DONT_CARE"),
    .read_during_write_mode_port_b ("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized        ("FALSE"),
    .init_file                     (INIT_FILE)
  ) ram_i (
    .clock0        (clk),
    .address_a     (addr_a), .data_a(data_a), .byteena_a(byteena_a), .wren_a(wren_a), .q_a(q_a),
    .address_b     (addr_b), .data_b(32'd0),  .byteena_b(4'b1111),   .wren_b(1'b0),   .q_b(q_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule

module cdf_display_m10k (
  input  wire        clk,
  input  wire [11:0] addr_a,
  input  wire [7:0]  data_a,
  input  wire        wren_a,
  output wire [7:0]  q_a,
  input  wire [11:0] addr_b,
  output wire [7:0]  q_b
);
  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (8), .widthad_a(12), .numwords_a(4096), .width_byteena_a(1),
    .width_b                       (8), .widthad_b(12), .numwords_b(4096), .width_byteena_b(1),
    .outdata_reg_a                 ("CLOCK0"),
    .outdata_reg_b                 ("CLOCK0"),
    .address_reg_b                 ("CLOCK0"),
    .indata_reg_b                  ("CLOCK0"),
    .wrcontrol_wraddress_reg_b     ("CLOCK0"),
    .byteena_reg_b                 ("CLOCK0"),
    .read_during_write_mode_port_a ("DONT_CARE"),
    .read_during_write_mode_port_b ("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized        ("FALSE")
  ) ram_i (
    .clock0        (clk),
    .address_a     (addr_a), .data_a(data_a), .byteena_a(1'b1), .wren_a(wren_a), .q_a(q_a),
    .address_b     (addr_b), .data_b(8'd0),    .byteena_b(1'b1), .wren_b(1'b0),  .q_b(q_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule
`default_nettype wire
