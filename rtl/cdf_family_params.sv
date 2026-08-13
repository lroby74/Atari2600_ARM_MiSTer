//============================================================================
// cdf_family_params.sv — Parameter lookup for the CDF family variants.
// Values taken verbatim from Stella (src/emucore/CartCDF.cxx setupVersion()
// and Thumbulator.cxx BX-to-ARM callback interception):
//
//              ds_base  inc_base  wf_base  amp   jmask  prog_off
//   CDF0       0x06E0   0x0768    0x07F0   0x22  0xFF   4KB
//   CDF1       0x00A0   0x0128    0x01B0   0x22  0xFF   4KB
//   CDFJ       0x0098   0x0124    0x01B0   0x23  0xFE   4KB
//   CDFJ+      0x0098   0x0124    0x01B0   0x23  0xFE   2KB
//
// family_sel comes from detect2600 (00=CDF0/1, 01=CDFJ, 10=CDFJ+). The 00
// code cannot distinguish CDF0 from CDF1; CDF1 values are used because every
// released CDF0/1 title (e.g. Draconian) ships the CDF1 driver. CDF0 is a
// couple of early demos only.
//
// cb0..cb3 are the Thumbulator "pc" values (BX instruction address + 4, due
// to ARM pipelining) at which _SetNote/_ResetWave/_GetWavePtr/_SetWaveSize
// BX-to-ARM-mode calls are intercepted:
//   CDF0        : 0x6E2/0x6E6/0x6EA/0x6EE  +4
//   CDF1/J/J+   : 0x752/0x756/0x75A/0x75E  +4
//============================================================================

`default_nettype none

module cdf_family_params (
    input  wire [1:0]  family_sel,    // 00=CDF0/1, 01=CDFJ, 10=CDFJ+
    output reg  [15:0] ds_base,       // Harmony RAM offset of datastream pointers
    output reg  [15:0] ds_inc_base,   // Harmony RAM offset of datastream increments
    output reg  [15:0] wf_base,       // Harmony RAM offset of waveform pointers
    output reg  [7:0]  amp_stream,    // AMPLITUDE stream index (also datastream limit)
    output reg  [7:0]  jump_mask,     // FASTJMP stream index mask
    output reg         is_plus,       // 1 = CDFJ+ (16.16 pointers, 2KB program base)
    output reg  [2:0]  start_bank,    // reset bank (CDF: 6, CDFJ+: 0)
    output reg  [15:0] prog_off,      // byte offset of 6502 program image in ROM
    output reg  [31:0] cb0,           // _SetNote intercept pc
    output reg  [31:0] cb1,           // _ResetWave intercept pc
    output reg  [31:0] cb2,           // _GetWavePtr intercept pc
    output reg  [31:0] cb3            // _SetWaveSize intercept pc
);

    always @* begin
        case (family_sel)
            2'b01: begin // CDFJ
                ds_base     = 16'h0098;
                ds_inc_base = 16'h0124;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h23;
                jump_mask   = 8'hfe;
                is_plus     = 1'b0;
                start_bank  = 3'd6;
                prog_off    = 16'h1000;
                cb0         = 32'h00000756;
                cb1         = 32'h0000075a;
                cb2         = 32'h0000075e;
                cb3         = 32'h00000762;
            end
            2'b10: begin // CDFJ+
                ds_base     = 16'h0098;
                ds_inc_base = 16'h0124;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h23;
                jump_mask   = 8'hfe;
                is_plus     = 1'b1;
                start_bank  = 3'd0;
                prog_off    = 16'h0800;
                cb0         = 32'h00000756;
                cb1         = 32'h0000075a;
                cb2         = 32'h0000075e;
                cb3         = 32'h00000762;
            end
            default: begin // 00 = CDF0/1 -> CDF1 driver layout
                ds_base     = 16'h00a0;
                ds_inc_base = 16'h0128;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h22;
                jump_mask   = 8'hff;
                is_plus     = 1'b0;
                start_bank  = 3'd6;
                prog_off    = 16'h1000;
                cb0         = 32'h00000756;
                cb1         = 32'h0000075a;
                cb2         = 32'h0000075e;
                cb3         = 32'h00000762;
            end
        endcase
    end

endmodule
