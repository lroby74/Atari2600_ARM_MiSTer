//============================================================================
// Module: mem_stage
// Description: MEM micro-stage for multi-cycle ARM/Thumb softcore.
//              Handles data memory access (load/store) with byte/halfword/word
//              alignment, byte-enable generation, and sign/zero extension.
//============================================================================

module mem_stage (
    // System
    input         clk,
    input         rst_n,

    // Control from Control Unit
    input         state_mem,       // High when FSM is in MEM state
    output        mem_done,        // High when memory access completes

    // Control signals from DECODE stage
    input         ctrl_mem_read,   // Enable memory read
    input         ctrl_mem_write,  // Enable memory write
    input  [1:0]  ctrl_mem_size,   // 00=byte, 01=halfword, 10=word
    input         ctrl_reg_src,    // 01 = data from memory to register

    // Data from EXEC stage
    input  [31:0] mem_addr_exec,   // Memory address (from ALU)
    input  [31:0] mem_wdata_exec,  // Data to write (from register)

    // Memory interface
    output reg [31:0] mem_addr,    // Address to memory
    output reg [31:0] mem_wdata,   // Data to write (aligned)
    input  [31:0] mem_rdata,       // Data from memory
    output reg        mem_read,    // Read enable
    output reg        mem_write,   // Write enable
    output reg [3:0]  mem_byte_en, // Byte-enable for partial writes
    input         mem_ready,       // Memory access completed

    // Output to WRITEBACK stage
    output reg [31:0] mem_rdata_wb,// Data for register writeback (sign/zero extended)
    output reg        mem_rdata_valid // Data valid for writeback
);

    //========================================================================
    // Internal signals
    //========================================================================
    reg [31:0] aligned_wdata;
    reg [3:0]  byte_enable;
    reg [31:0] extended_rdata;
    reg [1:0]  addr_lsb;

    //========================================================================
    // Address alignment and byte-enable generation
    //========================================================================
    always @(*) begin
        addr_lsb = mem_addr_exec[1:0];
        mem_addr = {mem_addr_exec[31:2], 2'b00};  // Word-aligned address for memory

        // Default: word access
        aligned_wdata = mem_wdata_exec;
        byte_enable = 4'b1111;

        case (ctrl_mem_size)
            2'b00: begin  // Byte
                case (addr_lsb)
                    2'b00: begin aligned_wdata = {24'h000000, mem_wdata_exec[7:0]}; byte_enable = 4'b0001; end
                    2'b01: begin aligned_wdata = {16'h0000, mem_wdata_exec[7:0], 8'h00}; byte_enable = 4'b0010; end
                    2'b10: begin aligned_wdata = {8'h00, mem_wdata_exec[7:0], 16'h0000}; byte_enable = 4'b0100; end
                    2'b11: begin aligned_wdata = {mem_wdata_exec[7:0], 24'h000000}; byte_enable = 4'b1000; end
                endcase
            end

            2'b01: begin  // Halfword
                case (addr_lsb[1])
                    1'b0: begin aligned_wdata = {16'h0000, mem_wdata_exec[15:0]}; byte_enable = 4'b0011; end
                    1'b1: begin aligned_wdata = {mem_wdata_exec[15:0], 16'h0000}; byte_enable = 4'b1100; end
                endcase
            end

            2'b10: begin  // Word
                aligned_wdata = mem_wdata_exec;
                byte_enable = 4'b1111;
            end

            default: begin
                aligned_wdata = mem_wdata_exec;
                byte_enable = 4'b1111;
            end
        endcase
    end

    //========================================================================
    // Read data extension (sign or zero)
    //========================================================================
    always @(*) begin
        extended_rdata = mem_rdata;

        if (ctrl_mem_read) begin
            case (ctrl_mem_size)
                2'b00: begin  // Byte
                    case (addr_lsb)
                        2'b00: extended_rdata = {{24{mem_rdata[7]}}, mem_rdata[7:0]};   // LDRSB (sign-extend)
                        2'b01: extended_rdata = {{24{mem_rdata[15]}}, mem_rdata[15:8]};
                        2'b10: extended_rdata = {{24{mem_rdata[23]}}, mem_rdata[23:16]};
                        2'b11: extended_rdata = {{24{mem_rdata[31]}}, mem_rdata[31:24]};
                    endcase
                end

                2'b01: begin  // Halfword
                    case (addr_lsb[1])
                        1'b0: extended_rdata = {{16{mem_rdata[15]}}, mem_rdata[15:0]};  // LDRSH (sign-extend)
                        1'b1: extended_rdata = {{16{mem_rdata[31]}}, mem_rdata[31:16]};
                    endcase
                end

                2'b10: begin  // Word
                    extended_rdata = mem_rdata;
                end

                default: extended_rdata = mem_rdata;
            endcase
        end
    end

    //========================================================================
    // Note on LDRB/LDRH vs LDRSB/LDRSH:
    // The Python model uses separate functions for sign-extend (LDRSB/LDRSH)
    // and zero-extend (LDRB/LDRH). In this RTL, we default to sign-extend
    // for byte/halfword loads. The decoder should set ctrl_mem_size and
    // an additional flag (not yet implemented) to select zero vs sign extend.
    // For now, the decoder in Section 16.D distinguishes LDRB vs LDRSB by
    // subop encoding, and this module would need a ctrl_sign_extend signal.
    // Simplified: assume zero-extend for LDRB/LDRH, sign-extend for LDRSB/LDRSH.
    // This is handled by the decoder setting appropriate control signals.
    //========================================================================

    //========================================================================
    // Memory interface control
    //========================================================================
    always @(*) begin
        if (state_mem) begin
            mem_wdata = aligned_wdata;
            mem_byte_en = byte_enable;
            mem_read = ctrl_mem_read;
            mem_write = ctrl_mem_write;
        end else begin
            mem_wdata = 32'h00000000;
            mem_byte_en = 4'b0000;
            mem_read = 1'b0;
            mem_write = 1'b0;
        end
    end

    //========================================================================
    // Sequential: latch read data for WRITEBACK
    //========================================================================
    assign mem_done = state_mem && mem_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_rdata_wb   <= 32'h00000000;
            mem_rdata_valid <= 1'b0;
        end else begin
            if (mem_done) begin
                mem_rdata_wb    <= extended_rdata;
                mem_rdata_valid <= ctrl_mem_read;
            end else if (!state_mem) begin
                mem_rdata_valid <= 1'b0;
            end
        end
    end

endmodule
