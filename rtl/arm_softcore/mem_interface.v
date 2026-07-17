//============================================================================
// Module: mem_interface
// Description: Memory interface with bank switching, alignment, byte-enable.
//              Translates core memory requests to external bus.
//============================================================================

module mem_interface (
    input         clk,
    input         rst_n,

    // Core-side interface
    input         mem_en,          // Enable access
    input         mem_rw,          // 0=read, 1=write
    input  [31:0] mem_addr,        // Byte address
    input  [31:0] mem_wdata,       // Data to write (word)
    input  [1:0]  mem_size,        // 00=byte, 01=halfword, 10=word
    output [31:0] mem_rdata,       // Data read (word, aligned)
    output        mem_ready,       // Access completed

    // External bus
    output [31:0] bus_addr,
    output [31:0] bus_wdata,
    input  [31:0] bus_rdata,
    output        bus_read,
    output        bus_write,
    output [3:0]  bus_byte_en,
    input         bus_ready,

    // Bank switching (memory-mapped I/O)
    input         bank_wr_en,
    input  [7:0]  bank_wr_data,
    output [7:0]  bank_reg
);

    //========================================================================
    // Bank register
    //========================================================================
    reg [7:0] flash_bank;
    assign bank_reg = flash_bank;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            flash_bank <= 8'h00;
        else if (bank_wr_en)
            flash_bank <= bank_wr_data;
    end

    //========================================================================
    // Address translation for banked flash
    //========================================================================
    wire [31:0] translated_addr;
    wire        is_flash_window;

    assign is_flash_window = (mem_addr >= 32'h00000000) && (mem_addr < 32'h00004000);

    // Flash window: bank * 0x4000 + offset
    assign translated_addr = is_flash_window ?
        {14'b0, flash_bank, mem_addr[13:0]} : mem_addr;

    //========================================================================
    // Byte-enable and data alignment for writes
    //========================================================================
    reg [3:0]  byte_en;
    reg [31:0] aligned_wdata;
    reg [31:0] aligned_rdata;
    reg [1:0]  addr_lsb;

    always @(*) begin
        addr_lsb = mem_addr[1:0];
        byte_en = 4'b0000;
        aligned_wdata = 32'h00000000;

        if (mem_en && mem_rw) begin  // Write
            case (mem_size)
                2'b00: begin  // Byte
                    case (addr_lsb)
                        2'b00: begin byte_en = 4'b0001; aligned_wdata = {24'h0, mem_wdata[7:0]}; end
                        2'b01: begin byte_en = 4'b0010; aligned_wdata = {16'h0, mem_wdata[7:0], 8'h0}; end
                        2'b10: begin byte_en = 4'b0100; aligned_wdata = {8'h0, mem_wdata[7:0], 16'h0}; end
                        2'b11: begin byte_en = 4'b1000; aligned_wdata = {mem_wdata[7:0], 24'h0}; end
                    endcase
                end
                2'b01: begin  // Halfword
                    case (addr_lsb[1])
                        1'b0: begin byte_en = 4'b0011; aligned_wdata = {16'h0, mem_wdata[15:0]}; end
                        1'b1: begin byte_en = 4'b1100; aligned_wdata = {mem_wdata[15:0], 16'h0}; end
                    endcase
                end
                2'b10: begin  // Word
                    byte_en = 4'b1111;
                    aligned_wdata = mem_wdata;
                end
                default: begin
                    byte_en = 4'b1111;
                    aligned_wdata = mem_wdata;
                end
            endcase
        end
    end

    //========================================================================
    // Read data alignment and extension
    //========================================================================
    always @(*) begin
        aligned_rdata = bus_rdata;
        if (mem_en && !mem_rw) begin  // Read
            case (mem_size)
                2'b00: begin  // Byte (sign-extend)
                    case (addr_lsb)
                        2'b00: aligned_rdata = {{24{bus_rdata[7]}}, bus_rdata[7:0]};
                        2'b01: aligned_rdata = {{24{bus_rdata[15]}}, bus_rdata[15:8]};
                        2'b10: aligned_rdata = {{24{bus_rdata[23]}}, bus_rdata[23:16]};
                        2'b11: aligned_rdata = {{24{bus_rdata[31]}}, bus_rdata[31:24]};
                    endcase
                end
                2'b01: begin  // Halfword (sign-extend)
                    case (addr_lsb[1])
                        1'b0: aligned_rdata = {{16{bus_rdata[15]}}, bus_rdata[15:0]};
                        1'b1: aligned_rdata = {{16{bus_rdata[31]}}, bus_rdata[31:16]};
                    endcase
                end
                2'b10: begin  // Word
                    aligned_rdata = bus_rdata;
                end
            endcase
        end
    end

    //========================================================================
    // Bus outputs
    //========================================================================
    assign bus_addr = {translated_addr[31:2], 2'b00};
    assign bus_wdata = aligned_wdata;
    assign bus_read = mem_en && !mem_rw;
    assign bus_write = mem_en && mem_rw;
    assign bus_byte_en = byte_en;

    assign mem_rdata = aligned_rdata;
    assign mem_ready = bus_ready;

endmodule
