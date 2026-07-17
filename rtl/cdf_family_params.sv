//============================================================================
// cdf_family_params.sv — Parameter lookup table for CDF family variations
// (CDF0 / CDF1 / CDFJ / CDFJplus). Provides generic, non-hardcoded addresses,
// shifts, callback entry points, and bank defaults based on family_sel.
//============================================================================

`default_nettype none

module cdf_family_params (
    input  wire [1:0]  family_sel,    // 00=CDF0/1, 01=CDFJ, 10=CDFJ+
    output reg  [15:0] ds_base,       // Harmony RAM offset for datastream pointers
    output reg  [15:0] ds_inc_base,   // Harmony RAM offset for datastream increments
    output reg  [15:0] wf_base,       // Harmony RAM offset for waveform registers
    output reg  [7:0]  amp_stream,    // Audio amplitude stream index
    output reg  [7:0]  jump_mask,     // Fast-jump mask
    output reg         is_plus,       // 1 if CDFJ+ (24-bit/16-bit shift mode)
    output reg  [2:0]  start_bank,    // Initial bank on reset
    output reg  [15:0] prog_off,      // Program ROM offset
    output reg  [4:0]  mus_shift,     // Music counter shift amount
    output reg  [4:0]  mus_maskbit,   // Music counter mask bit
    output reg  [31:0] cb0,           // Callback 0 (_SetNote entry point)
    output reg  [31:0] cb1,           // Callback 1 (_ResetWave entry point)
    output reg  [31:0] cb2,           // Callback 2 (_GetWavePtr entry point)
    output reg  [31:0] cb3            // Callback 3 (_SetWaveSize entry point)
);

    always @* begin
        case (family_sel)
            2'b00: begin // Standard CDF0 / CDF1
                ds_base     = 16'h0080;
                ds_inc_base = 16'h0108;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h21;
                jump_mask   = 8'hff;
                is_plus     = 1'b0;
                start_bank  = 3'd6;
                prog_off    = 16'h0000;
                mus_shift   = 5'd20;
                mus_maskbit = 5'd28;
                cb0         = 32'h00000404;
                cb1         = 32'h00000414;
                cb2         = 32'h00000424;
                cb3         = 32'h00000434;
            end
            2'b01: begin // CDFJ
                ds_base     = 16'h0080;
                ds_inc_base = 16'h0108;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h21;
                jump_mask   = 8'hff;
                is_plus     = 1'b0;
                start_bank  = 3'd6;
                prog_off    = 16'h0000;
                mus_shift   = 5'd20;
                mus_maskbit = 5'd28;
                cb0         = 32'h00000404;
                cb1         = 32'h00000414;
                cb2         = 32'h00000424;
                cb3         = 32'h00000434;
            end
            2'b10: begin // CDFJ+ (Plus mode)
                ds_base     = 16'h0080;
                ds_inc_base = 16'h0108;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h21;
                jump_mask   = 8'hff;
                is_plus     = 1'b1;
                start_bank  = 3'd0;
                prog_off    = 16'h0000;
                mus_shift   = 5'd16;
                mus_maskbit = 5'd24;
                cb0         = 32'h00000404;
                cb1         = 32'h00000414;
                cb2         = 32'h00000424;
                cb3         = 32'h00000434;
            end
            default: begin
                ds_base     = 16'h0080;
                ds_inc_base = 16'h0108;
                wf_base     = 16'h01b0;
                amp_stream  = 8'h21;
                jump_mask   = 8'hff;
                is_plus     = 1'b0;
                start_bank  = 3'd6;
                prog_off    = 16'h0000;
                mus_shift   = 5'd20;
                mus_maskbit = 5'd28;
                cb0         = 32'h00000404;
                cb1         = 32'h00000414;
                cb2         = 32'h00000424;
                cb3         = 32'h00000434;
            end
        endcase
    end

endmodule
