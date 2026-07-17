//============================================================================
// Module: regfile
// Description: Register file for ARM/Thumb multi-cycle softcore.
//              16 registers (R0-R15), CPSR, 2 async read ports + 1 sync write.
//============================================================================

module regfile (
    input         clk,
    input         rst_n,

    // Read port A (asynchronous)
    input  [3:0]  rd_addr_a,
    output [31:0] rd_data_a,

    // Read port B (asynchronous)
    input  [3:0]  rd_addr_b,
    output [31:0] rd_data_b,

    // Read port C (PC, asynchronous)
    output [31:0] pc,

    // Write port (synchronous on rising edge)
    input         wr_en,
    input  [3:0]  wr_addr,
    input  [31:0] wr_data,

    // CPSR
    input         cpsr_wr_en,
    input  [4:0]  cpsr_wr_data,    // {N, Z, C, V, T}
    output [4:0]  cpsr,

    // PC write (from writeback stage)
    input         pc_wr_en,
    input  [31:0] pc_wr_data,

    // LR write (from writeback stage)
    input         lr_wr_en,
    input  [31:0] lr_wr_data,

    // Thumb mode (from CPSR[0])
    output        thumb_mode
);

    //========================================================================
    // Register storage
    //========================================================================
    reg [31:0] regs [0:14];        // R0-R14 (R15=PC is separate)
    reg [31:0] pc_reg;
    reg [4:0]  cpsr_reg;

    //========================================================================
    // Asynchronous read with PC/SP/LR special handling
    //========================================================================
    wire [31:0] r15_read = pc_reg + (cpsr_reg[0] ? 32'd4 : 32'd8);  // PC+4 (Thumb) or PC+8 (ARM)

    assign rd_data_a = (rd_addr_a == 4'd15) ? r15_read :
                       (rd_addr_a == 4'd13) ? regs[13] :
                       (rd_addr_a == 4'd14) ? regs[14] :
                       regs[rd_addr_a];

    assign rd_data_b = (rd_addr_b == 4'd15) ? r15_read :
                       (rd_addr_b == 4'd13) ? regs[13] :
                       (rd_addr_b == 4'd14) ? regs[14] :
                       regs[rd_addr_b];

    assign pc = pc_reg;
    assign cpsr = cpsr_reg;
    assign thumb_mode = cpsr_reg[0];

    //========================================================================
    // Synchronous write
    //========================================================================
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 15; i = i + 1)
                regs[i] <= 32'h00000000;
            pc_reg <= 32'h00000800;       // ROM base
            cpsr_reg <= 5'b00000;         // ARM mode, no flags
        end else begin
            // General register write (R0-R12)
            if (wr_en && wr_addr < 4'd13) begin
                regs[wr_addr] <= wr_data;
            end

            // SP write (R13)
            if (wr_en && wr_addr == 4'd13) begin
                regs[13] <= wr_data;
            end

            // LR write (R14) — from writeback stage or general write
            if (lr_wr_en) begin
                regs[14] <= lr_wr_data;
            end else if (wr_en && wr_addr == 4'd14) begin
                regs[14] <= wr_data;
            end

            // PC write (R15) — from writeback stage
            if (pc_wr_en) begin
                pc_reg <= pc_wr_data;
            end

            // CPSR update
            if (cpsr_wr_en) begin
                cpsr_reg <= cpsr_wr_data;
            end
        end
    end

endmodule
