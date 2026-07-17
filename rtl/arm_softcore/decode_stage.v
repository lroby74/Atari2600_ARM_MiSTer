//============================================================================
// Module: decode_stage
// Description: DECODE micro-stage for multi-cycle ARM/Thumb softcore.
//              Decodes Thumb-1 (16-bit) and minimal ARM (32-bit) instructions.
//              Generates control signals for EXEC, MEM, WRITEBACK stages.
//============================================================================

module decode_stage (
    // System
    input         clk,
    input         rst_n,

    // Control from Control Unit
    input         state_decode,    // High when FSM is in DECODE state
    output        decode_done,     // High when decode is complete
    output reg    decode_fault,    // Unrecognized opcode

    // Input from FETCH stage
    input  [31:0] ir,              // Instruction Register
    input         ir_valid,        // IR valid
    input         thumb_mode,      // 1=Thumb, 0=ARM

    // PC (for PC-relative operations)
    input  [31:0] pc,

    // Control signals to EXEC / MEM / WRITEBACK stages
    output reg [3:0]  ctrl_alu_op,
    output reg [1:0]  ctrl_alu_src_a,
    output reg [1:0]  ctrl_alu_src_b,
    output reg        ctrl_reg_write,
    output reg [1:0]  ctrl_reg_src,
    output reg        ctrl_mem_read,
    output reg        ctrl_mem_write,
    output reg [1:0]  ctrl_mem_size,
    output reg        ctrl_branch,
    output reg        ctrl_link,
    output reg [3:0]  ctrl_cond,
    output reg        ctrl_use_imm,
    output reg [1:0]  ctrl_shift_op,
    output reg [4:0]  ctrl_shift_amt,
    output reg        ctrl_update_flags,

    // Register addresses
    output reg [3:0]  ctrl_rd,     // Destination register
    output reg [3:0]  ctrl_rn,     // First source register
    output reg [3:0]  ctrl_rs,     // Second source register / shift reg
    output reg [31:0] ctrl_imm,    // Immediate value (sign/zero extended)

    // Special: high-register flag for Thumb
    output reg        ctrl_high_reg
);

    //========================================================================
    // Internal wires for Thumb instruction fields
    //========================================================================
    wire [15:0] op = ir[15:0];

    // Common Thumb fields
    wire [2:0]  thumb_rd  = op[2:0];
    wire [2:0]  thumb_rs  = op[5:3];
    wire [2:0]  thumb_rn  = op[8:6];
    wire [4:0]  thumb_imm5 = op[10:6];
    wire [7:0]  thumb_imm8 = op[7:0];
    wire [10:0] thumb_imm11 = op[10:0];
    wire [3:0]  thumb_cond = op[11:8];
    wire [3:0]  thumb_alu = op[9:6];

    //========================================================================
    // Decode done / fault
    //========================================================================
    assign decode_done = state_decode && ir_valid;

    //========================================================================
    // Combinational Decode Logic
    //========================================================================
    always @(*) begin
        // Default values (NOP-like)
        ctrl_alu_op      = 4'b0100;  // ADD
        ctrl_alu_src_a   = 2'b00;    // Rd
        ctrl_alu_src_b   = 2'b00;    // Rs
        ctrl_reg_write   = 1'b0;
        ctrl_reg_src     = 2'b00;    // ALU result
        ctrl_mem_read    = 1'b0;
        ctrl_mem_write   = 1'b0;
        ctrl_mem_size    = 2'b10;    // word
        ctrl_branch      = 1'b0;
        ctrl_link        = 1'b0;
        ctrl_cond        = 4'b1110;  // AL (always)
        ctrl_use_imm     = 1'b0;
        ctrl_shift_op    = 2'b00;    // LSL
        ctrl_shift_amt   = 5'b00000;
        ctrl_update_flags = 1'b0;
        ctrl_rd          = 4'b0000;
        ctrl_rn          = 4'b0000;
        ctrl_rs          = 4'b0000;
        ctrl_imm         = 32'h00000000;
        ctrl_high_reg    = 1'b0;
        decode_fault     = 1'b0;

        if (state_decode && ir_valid) begin
            if (thumb_mode) begin
                //================================================================
                // THUMB-1 DECODE
                //================================================================
                casez (op)
                    //------------------------------------------------------------
                    // 1. Shift by immediate: 000[op][offset5][Rs][Rd]
                    //------------------------------------------------------------
                    16'b000??_?????_???_???: begin
                        ctrl_shift_op    = op[12:11];
                        ctrl_shift_amt   = op[10:6];
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rs          = {1'b0, thumb_rs};
                        ctrl_rn          = {1'b0, thumb_rs};  // source = Rs
                        ctrl_alu_src_a   = 2'b00;  // Rd (destination, also source for shifts)
                        ctrl_alu_src_b   = 2'b10;  // shift amount
                        ctrl_reg_write   = 1'b1;
                        ctrl_update_flags = 1'b1;
                        ctrl_alu_op      = 4'b1101;  // MOV/shift
                    end

                    //------------------------------------------------------------
                    // 2. Add/subtract: 00011[I][op][Rn/off3][Rs][Rd]
                    //------------------------------------------------------------
                    16'b00011_?_?_???_???_???: begin
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rn          = {1'b0, thumb_rs};  // first operand = Rs
                        ctrl_rs          = {1'b0, thumb_rn};  // second operand = Rn or imm3
                        ctrl_alu_src_a   = 2'b01;  // Rn (which is Rs in encoding)
                        ctrl_use_imm     = op[10];  // I bit
                        if (op[10]) begin
                            // Immediate 3-bit
                            ctrl_alu_src_b = 2'b01;  // imm
                            ctrl_imm       = {29'b0, thumb_rn};
                        end else begin
                            // Register
                            ctrl_alu_src_b = 2'b00;  // Rs
                        end
                        ctrl_reg_write   = 1'b1;
                        ctrl_update_flags = 1'b1;
                        if (op[9]) begin
                            ctrl_alu_op = 4'b0010;  // SUB
                        end else begin
                            ctrl_alu_op = 4'b0100;  // ADD
                        end
                    end

                    //------------------------------------------------------------
                    // 3. MOV/CMP/ADD/SUB immediate: 001[op][Rd][offset8]
                    //------------------------------------------------------------
                    16'b001??_???_????????: begin
                        ctrl_rd          = {1'b0, op[10:8]};
                        ctrl_rn          = {1'b0, op[10:8]};  // for CMP/ADD/SUB
                        ctrl_alu_src_a   = 2'b01;  // Rn
                        ctrl_alu_src_b   = 2'b01;  // imm
                        ctrl_imm         = {24'b0, thumb_imm8};
                        ctrl_use_imm     = 1'b1;
                        case (op[12:11])
                            2'b00: begin  // MOV
                                ctrl_alu_op = 4'b1101;
                                ctrl_reg_write = 1'b1;
                                ctrl_update_flags = 1'b1;
                            end
                            2'b01: begin  // CMP
                                ctrl_alu_op = 4'b1010;  // CMP = SUB without write
                                ctrl_update_flags = 1'b1;
                            end
                            2'b10: begin  // ADD
                                ctrl_alu_op = 4'b0100;
                                ctrl_reg_write = 1'b1;
                                ctrl_update_flags = 1'b1;
                            end
                            2'b11: begin  // SUB
                                ctrl_alu_op = 4'b0010;
                                ctrl_reg_write = 1'b1;
                                ctrl_update_flags = 1'b1;
                            end
                        endcase
                    end

                    //------------------------------------------------------------
                    // 4. ALU operations: 010000[op][Rs][Rd]
                    //------------------------------------------------------------
                    16'b010000_????_???_???: begin
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rn          = {1'b0, thumb_rd};  // first operand = Rd
                        ctrl_rs          = {1'b0, thumb_rs};
                        ctrl_alu_src_a   = 2'b00;  // Rd
                        ctrl_alu_src_b   = 2'b00;  // Rs
                        ctrl_reg_write   = (thumb_alu != 4'b1000 && thumb_alu != 4'b1010 && thumb_alu != 4'b1011);
                        ctrl_update_flags = 1'b1;
                        case (thumb_alu)
                            4'b0000: ctrl_alu_op = 4'b0000;  // AND
                            4'b0001: ctrl_alu_op = 4'b0001;  // EOR
                            4'b0010: ctrl_alu_op = 4'b1101;  // LSL (shift)
                            4'b0011: ctrl_alu_op = 4'b1101;  // LSR
                            4'b0100: ctrl_alu_op = 4'b1101;  // ASR
                            4'b0101: ctrl_alu_op = 4'b0101;  // ADC
                            4'b0110: ctrl_alu_op = 4'b0110;  // SBC
                            4'b0111: ctrl_alu_op = 4'b1101;  // ROR
                            4'b1000: ctrl_alu_op = 4'b0000;  // TST (AND, no write)
                            4'b1001: ctrl_alu_op = 4'b0010;  // NEG (RSB)
                            4'b1010: ctrl_alu_op = 4'b1010;  // CMP
                            4'b1011: ctrl_alu_op = 4'b0100;  // CMN
                            4'b1100: ctrl_alu_op = 4'b1100;  // ORR
                            4'b1101: ctrl_alu_op = 4'b1111;  // MUL (simplified)
                            4'b1110: ctrl_alu_op = 4'b1110;  // BIC
                            4'b1111: ctrl_alu_op = 4'b1111;  // MVN
                        endcase
                    end

                    //------------------------------------------------------------
                    // 5. High register / BX: 010001[op][H1][H2][Rs][Rd]
                    //------------------------------------------------------------
                    16'b010001_??_?_?_???_???: begin
                        ctrl_high_reg    = 1'b1;
                        ctrl_rd          = {op[7], thumb_rd};
                        ctrl_rs          = {op[6], thumb_rs};
                        ctrl_rn          = {op[7], thumb_rd};
                        case (op[9:8])
                            2'b00: begin  // ADD high
                                ctrl_alu_op = 4'b0100;
                                ctrl_reg_write = 1'b1;
                            end
                            2'b01: begin  // CMP high
                                ctrl_alu_op = 4'b1010;
                                ctrl_update_flags = 1'b1;
                            end
                            2'b10: begin  // MOV high
                                ctrl_alu_op = 4'b1101;
                                ctrl_reg_write = 1'b1;
                            end
                            2'b11: begin  // BX
                                ctrl_branch = 1'b1;
                                ctrl_alu_src_a = 2'b00;
                                ctrl_alu_src_b = 2'b00;
                            end
                        endcase
                    end

                    //------------------------------------------------------------
                    // 6. PC-relative LDR: 01001[Rd][word8]
                    //------------------------------------------------------------
                    16'b01001_???_????????: begin
                        ctrl_rd          = {1'b0, op[10:8]};
                        ctrl_alu_src_a   = 2'b10;  // PC
                        ctrl_alu_src_b   = 2'b01;  // imm
                        ctrl_imm         = {22'b0, op[7:0], 2'b00};  // word8 * 4
                        ctrl_alu_op      = 4'b0100;  // ADD
                        ctrl_reg_write   = 1'b1;
                        ctrl_reg_src     = 2'b01;  // from memory
                        ctrl_mem_read    = 1'b1;
                        ctrl_mem_size    = 2'b10;  // word
                    end

                    //------------------------------------------------------------
                    // 7. Register-offset load/store: 0101[op][Ro][Rb][Rd]
                    //------------------------------------------------------------
                    16'b0101_????_???_???_???: begin
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rn          = {1'b0, thumb_rs};  // Rb (base)
                        ctrl_rs          = {1'b0, thumb_rn};  // Ro (offset)
                        ctrl_alu_src_a   = 2'b01;  // Rb
                        ctrl_alu_src_b   = 2'b00;  // Ro
                        ctrl_alu_op      = 4'b0100;  // ADD (addr = Rb + Ro)
                        case (op[11:9])
                            3'b000: begin ctrl_mem_write = 1'b1; ctrl_mem_size = 2'b10; end  // STR
                            3'b001: begin ctrl_mem_write = 1'b1; ctrl_mem_size = 2'b01; end  // STRH
                            3'b010: begin ctrl_mem_write = 1'b1; ctrl_mem_size = 2'b00; end  // STRB
                            3'b011: begin ctrl_mem_read = 1'b1; ctrl_mem_size = 2'b00; ctrl_reg_write = 1'b1; ctrl_reg_src = 2'b01; end  // LDSB
                            3'b100: begin ctrl_mem_read = 1'b1; ctrl_mem_size = 2'b10; ctrl_reg_write = 1'b1; ctrl_reg_src = 2'b01; end  // LDR
                            3'b101: begin ctrl_mem_read = 1'b1; ctrl_mem_size = 2'b01; ctrl_reg_write = 1'b1; ctrl_reg_src = 2'b01; end  // LDRH
                            3'b110: begin ctrl_mem_read = 1'b1; ctrl_mem_size = 2'b00; ctrl_reg_write = 1'b1; ctrl_reg_src = 2'b01; end  // LDRB
                            3'b111: begin ctrl_mem_read = 1'b1; ctrl_mem_size = 2'b01; ctrl_reg_write = 1'b1; ctrl_reg_src = 2'b01; end  // LDSH
                        endcase
                    end

                    //------------------------------------------------------------
                    // 8. Immediate offset load/store word/byte: 011[B][L][offset5][Rb][Rd]
                    //------------------------------------------------------------
                    16'b0110_?_?_?????_???_???: begin
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rn          = {1'b0, thumb_rs};  // Rb
                        ctrl_alu_src_a   = 2'b01;  // Rb
                        ctrl_alu_src_b   = 2'b01;  // imm
                        if (op[12])  // B=1 (byte)
                            ctrl_imm = {27'b0, thumb_imm5};
                        else  // B=0 (word)
                            ctrl_imm = {25'b0, thumb_imm5, 2'b00};
                        ctrl_alu_op = 4'b0100;
                        if (op[11]) begin  // L=1 (load)
                            ctrl_mem_read = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_reg_src = 2'b01;
                            ctrl_mem_size = op[12] ? 2'b00 : 2'b10;
                        end else begin  // L=0 (store)
                            ctrl_mem_write = 1'b1;
                            ctrl_mem_size = op[12] ? 2'b00 : 2'b10;
                        end
                    end

                    //------------------------------------------------------------
                    // 9. Immediate offset halfword: 1000[L][offset5][Rb][Rd]
                    //------------------------------------------------------------
                    16'b1000_?_?????_???_???: begin
                        ctrl_rd          = {1'b0, thumb_rd};
                        ctrl_rn          = {1'b0, thumb_rs};
                        ctrl_alu_src_a   = 2'b01;
                        ctrl_alu_src_b   = 2'b01;
                        ctrl_imm         = {26'b0, thumb_imm5, 1'b0};
                        ctrl_alu_op      = 4'b0100;
                        if (op[11]) begin  // L=1
                            ctrl_mem_read = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_reg_src = 2'b01;
                        end else begin
                            ctrl_mem_write = 1'b1;
                        end
                        ctrl_mem_size = 2'b01;  // halfword
                    end

                    //------------------------------------------------------------
                    // 10. SP-relative load/store: 1001[L][Rd][word8]
                    //------------------------------------------------------------
                    16'b1001_?_???_????????: begin
                        ctrl_rd          = {1'b0, op[10:8]};
                        ctrl_alu_src_a   = 2'b11;  // SP (special encoding)
                        ctrl_alu_src_b   = 2'b01;
                        ctrl_imm         = {22'b0, op[7:0], 2'b00};
                        ctrl_alu_op      = 4'b0100;
                        if (op[11]) begin
                            ctrl_mem_read = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_reg_src = 2'b01;
                        end else begin
                            ctrl_mem_write = 1'b1;
                        end
                        ctrl_mem_size = 2'b10;
                    end

                    //------------------------------------------------------------
                    // 11. ADD PC/SP immediate: 1010[SP][Rd][word8]
                    //------------------------------------------------------------
                    16'b1010_?_???_????????: begin
                        ctrl_rd          = {1'b0, op[10:8]};
                        if (op[11]) begin  // SP
                            ctrl_alu_src_a = 2'b11;  // SP
                        end else begin  // PC
                            ctrl_alu_src_a = 2'b10;  // PC
                        end
                        ctrl_alu_src_b   = 2'b01;
                        ctrl_imm         = {22'b0, op[7:0], 2'b00};
                        ctrl_alu_op      = 4'b0100;
                        ctrl_reg_write   = 1'b1;
                    end

                    //------------------------------------------------------------
                    // 12. ADD/SUB SP immediate: 10110000[S][imm7]
                    //------------------------------------------------------------
                    16'b10110000_?_???????: begin
                        ctrl_rd          = 4'd13;  // SP
                        ctrl_rn          = 4'd13;
                        ctrl_alu_src_a   = 2'b11;  // SP
                        ctrl_alu_src_b   = 2'b01;
                        ctrl_imm         = {23'b0, op[6:0], 2'b00};
                        ctrl_alu_op      = op[7] ? 4'b0010 : 4'b0100;  // SUB : ADD
                        ctrl_reg_write   = 1'b1;
                    end

                    //------------------------------------------------------------
                    // 13. PUSH / POP: 1011[L][R][rlist]
                    //------------------------------------------------------------
                    16'b1011_?_?_?_????????: begin
                        // Simplified: treat as multi-cycle memory block operation
                        // Full implementation requires multi-cycle iteration in control unit
                        ctrl_mem_size = 2'b10;
                        if (op[11]) begin  // POP (LDMIA)
                            ctrl_mem_read = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_reg_src = 2'b01;
                        end else begin  // PUSH (STMDB)
                            ctrl_mem_write = 1'b1;
                        end
                    end

                    //------------------------------------------------------------
                    // 14. STMIA / LDMIA: 1100[L][Rb][rlist]
                    //------------------------------------------------------------
                    16'b1100_?_?_????????: begin
                        ctrl_rn = {1'b0, op[10:8]};  // Rb
                        ctrl_mem_size = 2'b10;
                        if (op[11]) begin  // LDMIA
                            ctrl_mem_read = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_reg_src = 2'b01;
                        end else begin  // STMIA
                            ctrl_mem_write = 1'b1;
                        end
                    end

                    //------------------------------------------------------------
                    // 15. Conditional branch: 1101[cond][offset8]
                    //------------------------------------------------------------
                    16'b1101_????_????????: begin
                        ctrl_branch = 1'b1;
                        ctrl_cond   = op[11:8];
                        ctrl_alu_src_a = 2'b10;  // PC
                        ctrl_alu_src_b = 2'b01;
                        ctrl_imm = {{23{op[7]}}, op[7:0], 1'b0};  // sign-extended * 2
                        ctrl_alu_op = 4'b0100;  // ADD
                    end

                    //------------------------------------------------------------
                    // 16. Unconditional branch: 11100[offset11]
                    //------------------------------------------------------------
                    16'b11100_???????????: begin
                        ctrl_branch = 1'b1;
                        ctrl_cond   = 4'b1110;  // AL
                        ctrl_alu_src_a = 2'b10;  // PC
                        ctrl_alu_src_b = 2'b01;
                        ctrl_imm = {{20{op[10]}}, op[10:0], 1'b0};
                        ctrl_alu_op = 4'b0100;
                    end

                    //------------------------------------------------------------
                    // 17-18. BL first/second half: 11110/11111[offset11]
                    //------------------------------------------------------------
                    16'b1111?_???????????: begin
                        if (!op[12]) begin  // BL first half (11110)
                            ctrl_link = 1'b1;
                            ctrl_reg_write = 1'b1;
                            ctrl_rd = 4'd14;  // LR
                            ctrl_reg_src = 2'b10;  // PC+4
                        end else begin  // BL second half (11111)
                            ctrl_branch = 1'b1;
                            ctrl_alu_src_a = 2'b10;  // PC
                            ctrl_alu_src_b = 2'b01;
                            ctrl_imm = {{20{op[10]}}, op[10:0], 1'b0};
                            ctrl_alu_op = 4'b0100;
                        end
                    end

                    //------------------------------------------------------------
                    // Default: unrecognized opcode
                    //------------------------------------------------------------
                    default: begin
                        decode_fault = 1'b1;
                    end
                endcase
            end else begin
                //================================================================
                // ARM DECODE (minimal subset for bootstrap)
                //================================================================
                // ARM BX Rm:  cond(31:28) 0001_0010_1111_1111_1111_0001_Rm(3:0)
                if (ir[27:4] == 28'h012FFF1) begin
                    ctrl_branch = 1'b1;
                    ctrl_rs = ir[3:0];
                    ctrl_cond = ir[31:28];
                end
                // ARM ADD Rd, Rn, #imm (rotated): cond 00_1_0100_0_Rn_Rd_rot_imm8
                else if (ir[27:26] == 2'b00 && ir[25] == 1'b1 && ir[24:21] == 4'b0100) begin
                    ctrl_alu_op = 4'b0100;
                    ctrl_rn = ir[19:16];
                    ctrl_rd = ir[15:12];
                    ctrl_alu_src_a = 2'b01;  // Rn
                    ctrl_alu_src_b = 2'b01;  // imm
                    ctrl_use_imm = 1'b1;
                    // Rotate immediate: imm8 ROR (rot*2)
                    ctrl_imm = ir[7:0];  // Simplified: decoder passes raw, EXEC rotates
                    ctrl_reg_write = 1'b1;
                end
                else begin
                    decode_fault = 1'b1;
                end
            end
        end
    end

endmodule
