//============================================================================
// thumb_decode.sv — Instruction Decoder for Thumb-1 16-bit Subset
// Provides parameter definitions and decoding masks used across thumb_core.
//============================================================================

`default_nettype none

module thumb_decode (
    input  wire [15:0] op,
    output reg  [6:0]  d_op,
    output reg  [3:0]  d_rd,
    output reg  [3:0]  d_rn,
    output reg  [3:0]  d_rm,
    output reg  [7:0]  d_imm8,
    output reg  [10:0] d_imm11,
    output reg         d_illegal
);

    // Opcode definitions matching thumb_core constants
    localparam [6:0] OP_ADC=7'd1,  OP_ADD1=7'd2,  OP_ADD2=7'd3,  OP_ADD3=7'd4,
      OP_ADD4=7'd5,  OP_ADD5=7'd6,  OP_ADD6=7'd7,  OP_ADD7=7'd8,  OP_AND=7'd9,
      OP_ASR1=7'd10, OP_ASR2=7'd11, OP_BEQ=7'd12, OP_BNE=7'd13, OP_BCS=7'd14,
      OP_BCC=7'd15,  OP_BMI=7'd16,  OP_BPL=7'd17, OP_BVS=7'd18, OP_BVC=7'd19,
      OP_BHI=7'd20,  OP_BLS=7'd21,  OP_BGE=7'd22, OP_BLT=7'd23, OP_BGT=7'd24,
      OP_BLE=7'd25,  OP_B2=7'd26,   OP_BIC=7'd27, OP_BKPT=7'd28,OP_BL=7'd29,
      OP_BLX1=7'd30, OP_BLX2=7'd31, OP_BX=7'd32,  OP_CMN=7'd33, OP_CMP1=7'd34,
      OP_CMP2=7'd35, OP_CMP3=7'd36, OP_CPS=7'd37, OP_CPY=7'd38, OP_EOR=7'd39,
      OP_LDMIA=7'd40,OP_LDR1=7'd41, OP_LDR2=7'd42,OP_LDR3=7'd43,OP_LDR4=7'd44,
      OP_LDRB1=7'd45,OP_LDRB2=7'd46,OP_LDRH1=7'd47,OP_LDRH2=7'd48,OP_LDRSB=7'd49,
      OP_LDRSH=7'd50,OP_LSL1=7'd51, OP_LSL2=7'd52,OP_LSR1=7'd53,OP_LSR2=7'd54,
      OP_MOV1=7'd55, OP_MOV2=7'd56, OP_MOV3=7'd57,OP_MUL=7'd58, OP_MVN=7'd59,
      OP_NEG=7'd60,  OP_ORR=7'd61,  OP_POP=7'd62, OP_PUSH=7'd63,OP_REV=7'd64,
      OP_REV16=7'd65,OP_REVSH=7'd66,OP_ROR=7'd67, OP_SBC=7'd68, OP_SETEND=7'd69,
      OP_STMIA=7'd70,OP_STR1=7'd71, OP_STR2=7'd72,OP_STR3=7'd73,OP_STRB1=7'd74,
      OP_STRB2=7'd75,OP_STRH1=7'd76,OP_STRH2=7'd77,OP_SUB1=7'd78,OP_SUB2=7'd79,
      OP_SUB3=7'd80, OP_SUB4=7'd81, OP_SXTB=7'd82,OP_SXTH=7'd83,OP_TST=7'd84,
      OP_UXTB=7'd85, OP_UXTH=7'd86, OP_UNDEF=7'd0;

    always @* begin
        d_rd      = op[2:0];
        d_rn      = op[5:3];
        d_rm      = op[8:6];
        d_imm8    = op[7:0];
        d_imm11   = op[10:0];
        d_illegal = 1'b0;

        if ((op & 16'hF800) == 16'h1800)      d_op = OP_ADD1;
        else if ((op & 16'hFE00) == 16'h1C00) d_op = OP_ADD2;
        else if ((op & 16'hE000) == 16'h2000) d_op = OP_MOV1;
        else if ((op & 16'hE000) == 16'h4000) d_op = OP_AND;
        else if ((op & 16'hF000) == 16'hD000) d_op = OP_BEQ;
        else                                  d_op = OP_UNDEF;
    end

endmodule
