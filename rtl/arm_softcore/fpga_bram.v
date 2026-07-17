//============================================================================
// Module: fpga_bram
// Description: Synchronous Dual-Port / Single-Port Block RAM inferred for
//              Intel / Altera M10K (Cyclone V / MAX 10) FPGA architectures.
//              Implements 8 KB (2048 x 32-bit words) shared RAM mapped at
//              0x40000000 - 0x40001FFF with 4-bit byte-enable masking.
//============================================================================

`timescale 1ns/1ps

module fpga_bram #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 11,           // 2^11 = 2048 words = 8 KB
    parameter INIT_FILE  = ""            // Optional initial memory content (.hex or .mif)
)(
    input                   clk,
    input                   en,          // Memory access enable
    input                   we,          // Write enable
    input  [ADDR_WIDTH-1:0] addr,        // Word address (11 bits)
    input  [DATA_WIDTH-1:0] wdata,       // Write data (32 bits)
    input  [3:0]            byte_en,     // Byte enable mask (4 bits)
    output reg [DATA_WIDTH-1:0] rdata    // Read data output
);

    //========================================================================
    // Altera / Intel M10K Block RAM Attribute Inference
    //========================================================================
    (* ramstyle = "M10K" *) reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    //========================================================================
    // Optional Memory Initialization
    //========================================================================
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, ram);
        end
    end

    //========================================================================
    // Synchronous Read and Byte-Masked Write
    //========================================================================
    always @(posedge clk) begin
        if (en) begin
            if (we) begin
                if (byte_en[0]) ram[addr][7:0]   <= wdata[7:0];
                if (byte_en[1]) ram[addr][15:8]  <= wdata[15:8];
                if (byte_en[2]) ram[addr][23:16] <= wdata[23:16];
                if (byte_en[3]) ram[addr][31:24] <= wdata[31:24];
                // In Read-During-Write, return new data or old data (new data here)
                rdata <= wdata;
            end else begin
                rdata <= ram[addr];
            end
        end
    end

endmodule
