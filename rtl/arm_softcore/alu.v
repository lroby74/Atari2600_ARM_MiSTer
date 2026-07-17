//============================================================================
// Module: alu
// Description: Standalone ALU for ARM/Thumb multi-cycle softcore.
//              16 operations, barrel shifter, flag generation.
//============================================================================

module alu (
    // Operands
    input  [31:0] a,               // Operand A
    input  [31:0] b,               // Operand B (before shift)
    input  [4:0]  shift_amt,       // Shift amount
    input  [1:0]  shift_op,        // 00=LSL, 01=LSR, 10=ASR, 11=ROR

    // Control
    input  [3:0]  alu_op,          // Operation code
    input         use_carry,       // Use carry_in for ADC/SBC
    input         c_in,            // Carry in

    // Results
    output reg [31:0] result,
    output reg        n, z, c, v   // CPSR flags
);

    //========================================================================
    // Internal: shifted operand and shift carry
    //========================================================================
    reg [31:0] b_shifted;
    reg        shift_carry;

    //========================================================================
    // Barrel Shifter
    //========================================================================
    always @(*) begin
        if (shift_amt == 5'b00000) begin
            b_shifted = b;
            shift_carry = c_in;
        end else begin
            case (shift_op)
                2'b00: begin  // LSL
                    if (shift_amt >= 32) begin
                        b_shifted = 32'h00000000;
                        shift_carry = (shift_amt == 32) ? b[0] : 1'b0;
                    end else begin
                        b_shifted = b << shift_amt;
                        shift_carry = b[32 - shift_amt];
                    end
                end
                2'b01: begin  // LSR
                    if (shift_amt >= 32) begin
                        b_shifted = 32'h00000000;
                        shift_carry = (shift_amt == 32) ? b[31] : 1'b0;
                    end else begin
                        b_shifted = b >> shift_amt;
                        shift_carry = b[shift_amt - 1];
                    end
                end
                2'b10: begin  // ASR
                    if (shift_amt >= 32) begin
                        b_shifted = {32{b[31]}};
                        shift_carry = b[31];
                    end else begin
                        b_shifted = $signed(b) >>> shift_amt;
                        shift_carry = b[shift_amt - 1];
                    end
                end
                2'b11: begin  // ROR
                    b_shifted = (b >> shift_amt) | (b << (32 - shift_amt));
                    shift_carry = b[shift_amt - 1];
                end
            endcase
        end
    end

    //========================================================================
    // ALU Operations
    //========================================================================
    always @(*) begin
        // Defaults
        result = 32'h00000000;
        n = 1'b0;
        z = 1'b1;
        c = shift_carry;
        v = 1'b0;

        case (alu_op)
            // AND
            4'b0000: begin
                result = a & b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // EOR
            4'b0001: begin
                result = a ^ b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // SUB
            4'b0010: begin
                {c, result} = {1'b0, a} - {1'b0, b_shifted};
                c = ~c;  // ARM borrow is inverted carry
                v = ((a[31] != b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // RSB
            4'b0011: begin
                {c, result} = {1'b0, b_shifted} - {1'b0, a};
                c = ~c;
                v = ((b_shifted[31] != a[31]) && (b_shifted[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // ADD
            4'b0100: begin
                {c, result} = {1'b0, a} + {1'b0, b_shifted};
                v = ((a[31] == b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // ADC
            4'b0101: begin
                {c, result} = {1'b0, a} + {1'b0, b_shifted} + {31'h0, c_in};
                v = ((a[31] == b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // SBC
            4'b0110: begin
                {c, result} = {1'b0, a} - {1'b0, b_shifted} - {31'h0, ~c_in};
                c = ~c;
                v = ((a[31] != b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // RSC
            4'b0111: begin
                {c, result} = {1'b0, b_shifted} - {1'b0, a} - {31'h0, ~c_in};
                c = ~c;
                v = ((b_shifted[31] != a[31]) && (b_shifted[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // TST
            4'b1000: begin
                result = a & b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // TEQ
            4'b1001: begin
                result = a ^ b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // CMP
            4'b1010: begin
                {c, result} = {1'b0, a} - {1'b0, b_shifted};
                c = ~c;
                v = ((a[31] != b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // CMN
            4'b1011: begin
                {c, result} = {1'b0, a} + {1'b0, b_shifted};
                v = ((a[31] == b_shifted[31]) && (a[31] != result[31]));
                n = result[31];
                z = (result == 32'h00000000);
            end

            // ORR
            4'b1100: begin
                result = a | b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // MOV
            4'b1101: begin
                result = b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // BIC
            4'b1110: begin
                result = a & ~b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            // MVN
            4'b1111: begin
                result = ~b_shifted;
                n = result[31];
                z = (result == 32'h00000000);
            end

            default: begin
                result = 32'h00000000;
                n = 1'b0;
                z = 1'b1;
                c = 1'b0;
                v = 1'b0;
            end
        endcase
    end

endmodule
