//============================================================================
// cdc_sync2.sv - generic 2-flop level synchronizer for single-bit signals
// crossing between two unrelated (asynchronous) clock domains. Used on the
// thumb_core<->cdf_bridge boundary once thumb_core moves to clk_vid while
// cdf_bridge/6502/TIA stay on clk_sys (Phase B). SV-2005 subset, no reset
// needed on the sync flops themselves (a synchronizer must not be gated by
// an asynchronous reset that could reintroduce metastability risk; the
// signal settles to a safe value within 2 dst_clk cycles regardless).
//============================================================================
`default_nettype none
module cdc_sync2 (
  input  wire dst_clk,
  input  wire d,
  output reg  q
);
  reg q1;
  always @(posedge dst_clk) begin
    q1 <= d;
    q  <= q1;
  end
endmodule
`default_nettype wire
