//============================================================================
// Module: fpga_rom
// Description: Synchronous Block ROM / Program Flash window inferred for
//              Intel / Altera M10K FPGA architectures.
//              Maps 64 KB (16384 x 32-bit words) at 0x00000000 - 0x0000FFFF.
//              Initialized from ARM compilation hex image (e.g. testarm.hex).
//============================================================================

`timescale 1ns/1ps

module fpga_rom #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 14,           // 2^14 = 16384 words = 64 KB
    parameter INIT_FILE  = "testarm.hex" // Default ARM binary hex file
)(
    input                   clk,
    input                   en,          // Memory read enable
    input  [ADDR_WIDTH-1:0] addr,        // Word address (14 bits)
    output reg [DATA_WIDTH-1:0] rdata    // Read data output
);

    //========================================================================
    // Altera / Intel M10K Block ROM Attribute Inference
    //========================================================================
    (* ramstyle = "M10K" *) reg [DATA_WIDTH-1:0] rom [0:(1<<ADDR_WIDTH)-1];

    //========================================================================
    // Initial Memory Loading (Quartus synthesis supports $readmemh with hex/mif)
    //========================================================================
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, rom);
        end
    end

    //========================================================================
    // Synchronous Read Process
    //========================================================================
    always @(posedge clk) begin
        if (en) begin
            rdata <= rom[addr];
        end
    end

endmodule
