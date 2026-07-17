//============================================================================
// Module: writeback_stage
// Description: WRITEBACK micro-stage for multi-cycle ARM/Thumb softcore.
//              Writes ALU result or memory data back to register file.
//              Updates CPSR flags, PC, and LR as needed.
//============================================================================

module writeback_stage (
    // System
    input         clk,
    input         rst_n,

    // Control from Control Unit
    input         state_wb,        // High when FSM is in WRITEBACK state
    output        wb_done,         // High when writeback completes

    // Control signals from DECODE stage (latched)
    input         ctrl_reg_write,  // Enable register write
    input  [1:0]  ctrl_reg_src,    // Source: 00=ALU, 01=mem, 10=PC+4, 11=LR
    input         ctrl_update_flags,// Update CPSR flags
    input         ctrl_branch,     // Branch instruction
    input         ctrl_link,       // BL instruction
    input  [3:0]  ctrl_rd,         // Destination register

    // Data inputs
    input  [31:0] alu_result,      // ALU result from EXEC
    input  [31:0] mem_rdata_wb,    // Memory data from MEM
    input  [31:0] pc_plus_4,       // PC + 4 (for BL link)
    input  [4:0]  flags_out,       // {N,Z,C,V,T} from EXEC
    input         flags_valid,     // Flags are valid
    input         branch_taken,    // Branch condition passed
    input  [31:0] branch_target,   // Branch target address
    input         cond_passed,     // Condition passed (for conditional exec)

    // Outputs to register file
    output reg        reg_wr_en,
    output reg [3:0]  reg_wr_addr,
    output reg [31:0] reg_wr_data,

    // Outputs to PC (R15)
    output reg        pc_wr_en,
    output reg [31:0] pc_wr_data,

    // Outputs to LR (R14)
    output reg        lr_wr_en,
    output reg [31:0] lr_wr_data,

    // Outputs to CPSR
    output reg        cpsr_wr_en,
    output reg [4:0]  cpsr_wr_data,

    // Return sentinel detection
    output        return_detected
);

    //========================================================================
    // Internal signals
    //========================================================================
    reg [31:0] wb_data_sel;
    reg        do_reg_write;
    reg        do_pc_update;
    reg [31:0] pc_next;

    //========================================================================
    // Return sentinel: PC == 0xFFFF0000 or 0xFFFF0001
    //========================================================================
    assign return_detected = (alu_result[31:16] == 16'hFFFF) &&
                             (alu_result[15:1] == 15'b000000000000000);

    //========================================================================
    // Select writeback data source
    //========================================================================
    always @(*) begin
        case (ctrl_reg_src)
            2'b00: wb_data_sel = alu_result;      // ALU result
            2'b01: wb_data_sel = mem_rdata_wb;    // Memory data
            2'b10: wb_data_sel = pc_plus_4;       // PC+4 (for BL)
            2'b11: wb_data_sel = lr_wr_data;      // LR (not typical)
            default: wb_data_sel = alu_result;
        endcase
    end

    //========================================================================
    // PC next calculation
    //========================================================================
    always @(*) begin
        if (branch_taken && ctrl_branch)
            pc_next = branch_target;
        else
            pc_next = pc_plus_4;  // Normal sequential execution
    end

    //========================================================================
    // Writeback logic (combinational, registered on clock)
    //========================================================================
    always @(*) begin
        // Defaults: no writes
        reg_wr_en    = 1'b0;
        reg_wr_addr  = 4'b0000;
        reg_wr_data  = 32'h00000000;
        pc_wr_en     = 1'b0;
        pc_wr_data   = 32'h00000000;
        lr_wr_en     = 1'b0;
        lr_wr_data   = 32'h00000000;
        cpsr_wr_en   = 1'b0;
        cpsr_wr_data = 5'b00000;

        if (state_wb && cond_passed) begin
            // Register write (Rd)
            if (ctrl_reg_write && ctrl_rd != 4'b1111) begin
                reg_wr_en   = 1'b1;
                reg_wr_addr = ctrl_rd;
                reg_wr_data = wb_data_sel;
            end

            // PC update (R15) — always updated, either sequentially or by branch
            pc_wr_en   = 1'b1;
            pc_wr_data = pc_next;

            // LR update (R14) — for BL instructions
            if (ctrl_link) begin
                lr_wr_en   = 1'b1;
                lr_wr_data = pc_plus_4;
            end

            // CPSR flags update
            if (ctrl_update_flags && flags_valid) begin
                cpsr_wr_en   = 1'b1;
                cpsr_wr_data = flags_out;
            end
        end
    end

    //========================================================================
    // Writeback done signal
    //========================================================================
    assign wb_done = state_wb;

endmodule
