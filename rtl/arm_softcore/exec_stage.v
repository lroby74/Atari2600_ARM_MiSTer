//============================================================================
// Module: exec_stage
// Description: EXEC micro-stage for multi-cycle ARM/Thumb softcore.
//              Executes ALU operations, shifts, branch target calculation,
//              and generates condition flags (N, Z, C, V).
//============================================================================

module exec_stage (
    // System
    input         clk,
    input         rst_n,

    // Control from Control Unit
    input         state_exec,      // High when FSM is in EXEC state
    output        exec_done,       // High when EXEC completes

    // Control signals from DECODE stage
    input  [3:0]  ctrl_alu_op,     // ALU operation code
    input  [1:0]  ctrl_alu_src_a,  // Select operand A
    input  [1:0]  ctrl_alu_src_b,  // Select operand B
    input         ctrl_use_imm,    // Use immediate for operand B
    input  [1:0]  ctrl_shift_op,   // Shift type
    input  [4:0]  ctrl_shift_amt,  // Shift amount
    input         ctrl_update_flags,// Update CPSR flags
    input         ctrl_branch,     // Branch instruction
    input  [3:0]  ctrl_cond,       // ARM condition code
    input         ctrl_high_reg,   // High register operation

    // Data inputs (from regfile or PC)
    input  [31:0] reg_a_data,      // Register A value (Rn or Rd)
    input  [31:0] reg_b_data,      // Register B value (Rs)
    input  [31:0] pc_data,         // PC value
    input  [31:0] sp_data,         // SP value (R13)
    input  [31:0] ctrl_imm,        // Immediate value from decoder

    // CPSR input for condition evaluation
    input  [4:0]  cpsr_in,         // {N, Z, C, V, T}

    // Outputs to MEM / WRITEBACK stages
    output reg [31:0] alu_result,
    output reg [31:0] branch_target,
    output reg        branch_taken,
    output reg        cond_passed,
    output reg [4:0]  flags_out,    // {N, Z, C, V, T}
    output reg        flags_valid,

    // Memory address for load/store (passed to MEM stage)
    output reg [31:0] mem_addr_exec,
    output reg [31:0] mem_wdata_exec,
    output reg        mem_is_store
);

    //========================================================================
    // Internal signals
    //========================================================================
    reg [31:0] operand_a;
    reg [31:0] operand_b_raw;
    reg [31:0] operand_b_shifted;
    reg [31:0] alu_result_comb;
    reg        flag_n, flag_z, flag_c, flag_v;
    reg        cond_result;

    //========================================================================
    // Operand A selection
    //========================================================================
    always @(*) begin
        case (ctrl_alu_src_a)
            2'b00: operand_a = reg_a_data;   // Rd (also used as source for shifts)
            2'b01: operand_a = reg_a_data;   // Rn
            2'b10: operand_a = pc_data;      // PC
            2'b11: operand_a = sp_data;      // SP
            default: operand_a = reg_a_data;
        endcase
    end

    //========================================================================
    // Operand B selection (before shift)
    //========================================================================
    always @(*) begin
        if (ctrl_use_imm)
            operand_b_raw = ctrl_imm;
        else
            operand_b_raw = reg_b_data;
    end

    //========================================================================
    // Shifter (Thumb shift by immediate or register)
    //========================================================================
    always @(*) begin
        if (ctrl_shift_amt == 5'b00000 && !ctrl_use_imm) begin
            operand_b_shifted = operand_b_raw;
            flag_c = cpsr_in[1];  // Carry unchanged
        end else begin
            case (ctrl_shift_op)
                2'b00: begin  // LSL
                    if (ctrl_shift_amt >= 32) begin
                        operand_b_shifted = 32'h00000000;
                        flag_c = (ctrl_shift_amt == 32) ? operand_b_raw[0] : 1'b0;
                    end else begin
                        operand_b_shifted = operand_b_raw << ctrl_shift_amt;
                        flag_c = (ctrl_shift_amt == 0) ? cpsr_in[1] : operand_b_raw[32 - ctrl_shift_amt];
                    end
                end
                2'b01: begin  // LSR
                    if (ctrl_shift_amt >= 32) begin
                        operand_b_shifted = 32'h00000000;
                        flag_c = (ctrl_shift_amt == 32) ? operand_b_raw[31] : 1'b0;
                    end else begin
                        operand_b_shifted = operand_b_raw >> ctrl_shift_amt;
                        flag_c = (ctrl_shift_amt == 0) ? cpsr_in[1] : operand_b_raw[ctrl_shift_amt - 1];
                    end
                end
                2'b10: begin  // ASR
                    if (ctrl_shift_amt >= 32) begin
                        operand_b_shifted = {32{operand_b_raw[31]}};
                        flag_c = operand_b_raw[31];
                    end else begin
                        operand_b_shifted = $signed(operand_b_raw) >>> ctrl_shift_amt;
                        flag_c = (ctrl_shift_amt == 0) ? cpsr_in[1] : operand_b_raw[ctrl_shift_amt - 1];
                    end
                end
                2'b11: begin  // ROR
                    if (ctrl_shift_amt == 5'b00000) begin
                        operand_b_shifted = operand_b_raw;
                        flag_c = cpsr_in[1];
                    end else begin
                        operand_b_shifted = (operand_b_raw >> ctrl_shift_amt) | (operand_b_raw << (32 - ctrl_shift_amt));
                        flag_c = operand_b_raw[ctrl_shift_amt - 1];
                    end
                end
            endcase
        end
    end

    //========================================================================
    // ALU operations
    //========================================================================
    always @(*) begin
        flag_n = flag_n;  // latch defaults
        flag_z = flag_z;
        flag_v = flag_v;
        alu_result_comb = 32'h00000000;

        case (ctrl_alu_op)
            // AND
            4'b0000: begin
                alu_result_comb = operand_a & operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // EOR
            4'b0001: begin
                alu_result_comb = operand_a ^ operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // SUB / CMP
            4'b0010, 4'b1010: begin
                {flag_c, alu_result_comb} = {1'b0, operand_a} - {1'b0, operand_b_shifted};
                flag_c = !flag_c;  // ARM borrow is inverted carry
                flag_v = ((operand_a[31] != operand_b_shifted[31]) && (operand_a[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // RSB
            4'b0011: begin
                {flag_c, alu_result_comb} = {1'b0, operand_b_shifted} - {1'b0, operand_a};
                flag_c = !flag_c;
                flag_v = ((operand_b_shifted[31] != operand_a[31]) && (operand_b_shifted[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // ADD / CMN
            4'b0100, 4'b1011: begin
                {flag_c, alu_result_comb} = {1'b0, operand_a} + {1'b0, operand_b_shifted};
                flag_v = ((operand_a[31] == operand_b_shifted[31]) && (operand_a[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // ADC
            4'b0101: begin
                {flag_c, alu_result_comb} = {1'b0, operand_a} + {1'b0, operand_b_shifted} + {31'h0, cpsr_in[1]};
                flag_v = ((operand_a[31] == operand_b_shifted[31]) && (operand_a[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // SBC
            4'b0110: begin
                {flag_c, alu_result_comb} = {1'b0, operand_a} - {1'b0, operand_b_shifted} - {31'h0, ~cpsr_in[1]};
                flag_c = !flag_c;
                flag_v = ((operand_a[31] != operand_b_shifted[31]) && (operand_a[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // RSC
            4'b0111: begin
                {flag_c, alu_result_comb} = {1'b0, operand_b_shifted} - {1'b0, operand_a} - {31'h0, ~cpsr_in[1]};
                flag_c = !flag_c;
                flag_v = ((operand_b_shifted[31] != operand_a[31]) && (operand_b_shifted[31] != alu_result_comb[31]));
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // TST
            4'b1000: begin
                alu_result_comb = operand_a & operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // TEQ
            4'b1001: begin
                alu_result_comb = operand_a ^ operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // ORR
            4'b1100: begin
                alu_result_comb = operand_a | operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // MOV / shift operations
            4'b1101: begin
                alu_result_comb = operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // BIC
            4'b1110: begin
                alu_result_comb = operand_a & ~operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            // MVN / NEG
            4'b1111: begin
                alu_result_comb = ~operand_b_shifted;
                flag_n = alu_result_comb[31];
                flag_z = (alu_result_comb == 32'h00000000);
            end

            default: begin
                alu_result_comb = 32'h00000000;
                flag_n = 1'b0;
                flag_z = 1'b1;
                flag_c = 1'b0;
                flag_v = 1'b0;
            end
        endcase
    end

    //========================================================================
    // Condition evaluation (ARM condition codes)
    //========================================================================
    always @(*) begin
        case (ctrl_cond)
            4'b0000: cond_result = cpsr_in[2];                    // EQ (Z=1)
            4'b0001: cond_result = !cpsr_in[2];                   // NE (Z=0)
            4'b0010: cond_result = cpsr_in[1];                    // CS/HS (C=1)
            4'b0011: cond_result = !cpsr_in[1];                   // CC/LO (C=0)
            4'b0100: cond_result = cpsr_in[3];                    // MI (N=1)
            4'b0101: cond_result = !cpsr_in[3];                   // PL (N=0)
            4'b0110: cond_result = cpsr_in[0];                    // VS (V=1)
            4'b0111: cond_result = !cpsr_in[0];                   // VC (V=0)
            4'b1000: cond_result = cpsr_in[1] && !cpsr_in[2];    // HI (C=1 && Z=0)
            4'b1001: cond_result = !cpsr_in[1] || cpsr_in[2];    // LS (C=0 || Z=1)
            4'b1010: cond_result = (cpsr_in[3] == cpsr_in[0]);   // GE (N==V)
            4'b1011: cond_result = (cpsr_in[3] != cpsr_in[0]);   // LT (N!=V)
            4'b1100: cond_result = !cpsr_in[2] && (cpsr_in[3] == cpsr_in[0]); // GT
            4'b1101: cond_result = cpsr_in[2] || (cpsr_in[3] != cpsr_in[0]);  // LE
            4'b1110: cond_result = 1'b1;                          // AL (always)
            4'b1111: cond_result = 1'b0;                          // NV (never)
            default: cond_result = 1'b0;
        endcase
    end

    //========================================================================
    // Branch target calculation
    //========================================================================
    always @(*) begin
        if (ctrl_branch) begin
            // For Thumb: target = PC + 4 + offset (offset already includes *2)
            // For BX: target = Rs value
            if (ctrl_high_reg && ctrl_alu_op == 4'b1101)  // BX
                branch_target = {reg_b_data[31:1], 1'b0};  // Clear bit 0 for alignment
            else
                branch_target = alu_result_comb;  // PC-relative branch
            branch_taken = cond_result;
        end else begin
            branch_target = 32'h00000000;
            branch_taken = 1'b0;
        end
    end

    //========================================================================
    // Memory address and data for load/store
    //========================================================================
    always @(*) begin
        mem_addr_exec = alu_result_comb;  // Address = Rb + offset (calculated by ALU)
        mem_wdata_exec = reg_b_data;      // Data to store = Rd (or Rs depending on encoding)
        mem_is_store = 1'b0;              // Will be set by ctrl_mem_write in MEM stage
    end

    //========================================================================
    // Sequential: latch outputs on EXEC state
    //========================================================================
    assign exec_done = state_exec;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alu_result   <= 32'h00000000;
            flags_out    <= 5'b00000;
            flags_valid  <= 1'b0;
            cond_passed  <= 1'b0;
        end else begin
            if (state_exec) begin
                alu_result  <= alu_result_comb;
                flags_out   <= {flag_n, flag_z, flag_c, flag_v, cpsr_in[0]};  // {N,Z,C,V,T}
                flags_valid <= ctrl_update_flags;
                cond_passed <= cond_result;
            end
        end
    end

endmodule
