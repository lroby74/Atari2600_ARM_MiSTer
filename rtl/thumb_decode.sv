//============================================================================
// thumb_decode.sv — Instruction Decoder for Thumb-1 16-bit Subset
// Provides opcode and operand fields consumed by thumb_core.
//
// 2 ago 2026 - RISCRITTO DA CATENA A `casez`, comportamento INVARIATO.
//
// La versione precedente era una catena di 25 `else if` con maschere diverse
// (`(op & 16'hF800) == ...`, `FE00`, `FC00`, `FF00`, `F000`): il sintetizzatore
// deve valutarle in ordine e incatenare 25 mux in serie. Ma i pattern sono
// MUTUAMENTE ESCLUSIVI - verificato uno per uno - quindi la priorita' non
// serviva a nulla: era un mux piatto scritto come catena.
//
// Perche' conta: d_op/d_rd/d_rn/d_rm escono da qui e vanno al register file di
// thumb_core, e il cammino critico del design e' esattamente `ir -> r[]`
// (misurato: con la RAM Harmony a 32 KB il fit sfora a -0.091 ns proprio su
// ir[15] -> r[0][23]). Togliere 24 livelli di mux in serie da quel cono e' il
// primo intervento strutturale sensato.
//
// SICUREZZA: `tb/tb_decode_equiv.sv` confronta questa versione con la copia
// della precedente (`tb/thumb_decode_ref.sv`) su TUTTI e 65536 gli opcode a
// 16 bit, tutte e sette le uscite. Non e' un campione, e' l'intero dominio:
//   iverilog -g2012 -s tb_decode_equiv -o deq.vvp \
//       rtl/thumb_decode.sv tb/thumb_decode_ref.sv tb/tb_decode_equiv.sv
// Deve stampare "DECODER EQUIVALENTE" prima di qualunque sintesi.
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

    // Opcode definitions must match thumb_core.sv.
    localparam [6:0] OP_UNDEF=7'd0,
      OP_ADC=7'd1,  OP_ADD1=7'd2,  OP_ADD2=7'd3,  OP_ADD3=7'd4,
      OP_ADD4=7'd5,  OP_ADD5=7'd6,  OP_ADD6=7'd7,  OP_ADD7=7'd8,  OP_AND=7'd9,
      OP_ASR1=7'd10, OP_ASR2=7'd11, OP_BEQ=7'd12, OP_BNE=7'd13, OP_BCS=7'd14,
      OP_BCC=7'd15,  OP_BMI=7'd16,  OP_BPL=7'd17, OP_BVS=7'd18, OP_BVC=7'd19,
      OP_BHI=7'd20,  OP_BLS=7'd21,  OP_BGE=7'd22, OP_BLT=7'd23, OP_BGT=7'd24,
      OP_BLE=7'd25,  OP_B2=7'd26,   OP_BIC=7'd27, OP_BKPT=7'd28,OP_BL=7'd29,
      OP_BLX_THUMB=7'd30, OP_BLX_ARM=7'd31, OP_BLX2=7'd32, OP_BX=7'd33,
      OP_CMN=7'd34,  OP_CMP1=7'd35, OP_CMP2=7'd36, OP_CMP3=7'd37,OP_CPS=7'd38,
      OP_CPY=7'd39,  OP_EOR=7'd40,  OP_LDMIA=7'd41,OP_LDR1=7'd42, OP_LDR2=7'd43,
      OP_LDR3=7'd44, OP_LDR4=7'd45, OP_LDRB1=7'd46,OP_LDRB2=7'd47,OP_LDRH1=7'd48,
      OP_LDRH2=7'd49,OP_LDRSB=7'd50, OP_LDRSH=7'd51,OP_LSL1=7'd52, OP_LSL2=7'd53,
      OP_LSR1=7'd54, OP_LSR2=7'd55, OP_MOV1=7'd56, OP_MOV2=7'd57, OP_MOV3=7'd58,
      OP_MUL=7'd59,  OP_MVN=7'd60,  OP_NEG=7'd61,  OP_ORR=7'd62,  OP_POP=7'd63,
      OP_PUSH=7'd64, OP_REV=7'd65,  OP_REV16=7'd66,OP_REVSH=7'd67,OP_ROR=7'd68,
      OP_SBC=7'd69,  OP_SETEND=7'd70,OP_STMIA=7'd71,OP_STR1=7'd72, OP_STR2=7'd73,
      OP_STR3=7'd74, OP_STRB1=7'd75,OP_STRB2=7'd76,OP_STRH1=7'd77,OP_STRH2=7'd78,
      OP_SUB1=7'd79, OP_SUB2=7'd80, OP_SUB3=7'd81, OP_SUB4=7'd82, OP_SWI=7'd83,
      OP_SXTB=7'd84, OP_SXTH=7'd85, OP_TST=7'd86,  OP_UXTB=7'd87, OP_UXTH=7'd88;

    always @* begin
        d_op      = OP_UNDEF;
        d_rd      = {1'b0, op[2:0]};
        d_rn      = {1'b0, op[5:3]};
        d_rm      = {1'b0, op[8:6]};
        d_imm8    = op[7:0];
        d_imm11   = {6'd0, op[10:6]};
        d_illegal = 1'b0;

        // I pattern sono disgiunti: l'ordine e' quello della catena originale
        // solo per rendere il confronto riga per riga immediato.
        casez (op)
            // --- shift per immediato / add-sub (0x0000-0x1FFF) ---
            16'b00000???????????: begin d_op = OP_LSL1; d_rm = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b00001???????????: begin d_op = OP_LSR1; d_rm = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b00010???????????: begin d_op = OP_ASR1; d_rm = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b0001100?????????: begin d_op = OP_ADD3; d_rn = {1'b0, op[5:3]}; d_rm = {1'b0, op[8:6]}; end
            16'b0001101?????????: begin d_op = OP_SUB3; d_rn = {1'b0, op[5:3]}; d_rm = {1'b0, op[8:6]}; end
            16'b0001110?????????: begin d_op = OP_ADD1; d_rn = {1'b0, op[5:3]}; d_imm11 = {8'd0, op[8:6]}; end
            16'b0001111?????????: begin d_op = OP_SUB1; d_rn = {1'b0, op[5:3]}; d_imm11 = {8'd0, op[8:6]}; end

            // --- immediato a 8 bit (0x2000-0x3FFF) ---
            16'b00100???????????: begin d_op = OP_MOV1; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b00101???????????: begin d_op = OP_CMP1; d_rn = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b00110???????????: begin d_op = OP_ADD2; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b00111???????????: begin d_op = OP_SUB2; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end

            // --- ALU a registro (0x4000) ---
            16'b010000??????????: begin
                d_rn = {1'b0, op[2:0]}; d_rm = {1'b0, op[5:3]};
                case (op[9:6])
                    4'h0: d_op = OP_AND;
                    4'h1: d_op = OP_EOR;
                    4'h2: d_op = OP_LSL2;
                    4'h3: d_op = OP_LSR2;
                    4'h4: d_op = OP_ASR2;
                    4'h5: d_op = OP_ADC;
                    4'h6: d_op = OP_SBC;
                    4'h7: d_op = OP_ROR;
                    4'h8: d_op = OP_TST;
                    4'h9: d_op = OP_NEG;
                    4'ha: d_op = OP_CMP2;
                    4'hb: d_op = OP_CMN;
                    4'hc: d_op = OP_ORR;
                    4'hd: d_op = OP_MUL;
                    4'he: d_op = OP_BIC;
                    4'hf: d_op = OP_MVN;
                endcase
            end

            // --- registri alti + BX/BLX (0x4400) ---
            16'b010001??????????: begin
                d_rd = {op[7], op[2:0]}; d_rn = {op[7], op[2:0]}; d_rm = {op[6], op[5:3]};
                case (op[9:8])
                    2'b00: d_op = OP_ADD4;
                    2'b01: d_op = OP_CMP3;
                    2'b10: d_op = OP_MOV3;
                    default: d_op = op[7] ? OP_BLX2 : OP_BX;
                endcase
            end

            // --- LDR literal (0x4800) ---
            16'b01001???????????: begin d_op = OP_LDR3; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end

            // --- load/store con offset a registro (0x5000) ---
            16'b0101????????????: begin
                d_rn = {1'b0, op[5:3]}; d_rm = {1'b0, op[8:6]};
                case (op[11:9])
                    3'b000: d_op = OP_STR2;
                    3'b001: d_op = OP_STRH2;
                    3'b010: d_op = OP_STRB2;
                    3'b011: d_op = OP_LDRSB;
                    3'b100: d_op = OP_LDR2;
                    3'b101: d_op = OP_LDRH2;
                    3'b110: d_op = OP_LDRB2;
                    default: d_op = OP_LDRSH;
                endcase
            end

            // --- load/store con offset immediato (0x6000-0x8FFF) ---
            16'b01100???????????: begin d_op = OP_STR1;  d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b01101???????????: begin d_op = OP_LDR1;  d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b01110???????????: begin d_op = OP_STRB1; d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b01111???????????: begin d_op = OP_LDRB1; d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b10000???????????: begin d_op = OP_STRH1; d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end
            16'b10001???????????: begin d_op = OP_LDRH1; d_rn = {1'b0, op[5:3]}; d_imm11 = {6'd0, op[10:6]}; end

            // --- relativi a SP / PC (0x9000-0xAFFF) ---
            16'b10010???????????: begin d_op = OP_STR3; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b10011???????????: begin d_op = OP_LDR4; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b10100???????????: begin d_op = OP_ADD5; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end
            16'b10101???????????: begin d_op = OP_ADD6; d_rd = {1'b0, op[10:8]}; d_imm11 = {3'd0, op[7:0]}; end

            // --- gruppo 0xB: SP adjust, estensioni, PUSH/POP, REV, BKPT ---
            // I sottogruppi non coperti (B1/B3/B6/B7/B8/B9/BB/BF) restano UNDEF
            // come nella catena originale: CPS e SETEND sono ARMv6, impossibili
            // su ARM7TDMI, e non vanno aggiunti.
            16'b10110000????????: begin d_op = op[7] ? OP_SUB4 : OP_ADD7; d_imm11 = {4'd0, op[6:0]}; end
            16'b10110010????????: begin
                d_rm = {1'b0, op[5:3]};
                case (op[7:6])
                    2'b00: d_op = OP_SXTH;
                    2'b01: d_op = OP_SXTB;
                    2'b10: d_op = OP_UXTH;
                    default: d_op = OP_UXTB;
                endcase
            end
            16'b1011010?????????: d_op = OP_PUSH;
            16'b10111010????????: begin
                d_rm = {1'b0, op[5:3]};
                case (op[7:6])
                    2'b00: d_op = OP_REV;
                    2'b01: d_op = OP_REV16;
                    2'b11: d_op = OP_REVSH;
                    default: d_op = OP_UNDEF;
                endcase
            end
            16'b10111110????????: d_op = OP_BKPT;
            16'b1011110?????????: d_op = OP_POP;

            // --- LDMIA/STMIA (0xC000) ---
            16'b1100????????????: begin d_op = op[11] ? OP_LDMIA : OP_STMIA; d_rn = {1'b0, op[10:8]}; end

            // --- salti condizionati / SWI (0xD000). op[11:8]==0xE e' UNDEF. ---
            16'b1101????????????: begin
                if (op[11:8] == 4'hf)      d_op = OP_SWI;
                else if (op[11:8] != 4'he) d_op = OP_BEQ + {3'd0, op[11:8]};
            end

            // --- salti incondizionati e lunghi (0xE000-0xFFFF) ---
            16'b11100???????????: begin d_op = OP_B2;        d_imm11 = op[10:0]; end
            16'b11101???????????: begin d_op = OP_BLX_ARM;   d_imm11 = op[10:0]; end
            16'b11110???????????: begin d_op = OP_BL;        d_imm11 = op[10:0]; end
            16'b11111???????????: begin d_op = OP_BLX_THUMB; d_imm11 = op[10:0]; end

            default: ; // resta OP_UNDEF
        endcase

        if (d_op == OP_UNDEF) d_illegal = 1'b1;
    end

endmodule
`default_nettype wire
