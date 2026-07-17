// Cyclone-V M10K true-dual-port esplicito. Nessun fallback condizionale:
// l'istanza altsyncram e' sempre presente. Tutti i parametri di
// registrazione porta B sono forzati a CLOCK0 (il default impliciito
// Intel per BIDIR_DUAL_PORT e' CLOCK1: lasciarne anche uno solo di
// default causa Error 272006/287078/12152).
`default_nettype none
module harmony_m10k_tdp #(
  parameter INIT_FILE = ""
)(
  input  wire        clk,
  input  wire [10:0] addr_a,
  input  wire [31:0] data_a,
  input  wire [3:0]  byteena_a,
  input  wire        wren_a,
  output wire [31:0] q_a,
  input  wire [10:0] addr_b,
  input  wire [31:0] data_b,
  input  wire [3:0]  byteena_b,
  input  wire        wren_b,
  output wire [31:0] q_b
);
  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (32), .widthad_a(11), .numwords_a(2048), .width_byteena_a(4),
    .width_b                       (32), .widthad_b(11), .numwords_b(2048), .width_byteena_b(4),
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
    .address_b     (addr_b), .data_b(data_b), .byteena_b(byteena_b), .wren_b(wren_b), .q_b(q_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule
`default_nettype wire
