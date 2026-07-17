//============================================================================
// thumb_core.sv  —  core Thumb: fetch + decode(riusato) + execute + bus bridge
// (FASE 3). Basato su stella-emu/stella  src/emucore/Thumbulator.cxx
// (public domain / GPL, David Welch / F. Quimby). Nessun firmware proprietario.
// Stile SystemVerilog-2005, "Sorgelig": always @(posedge clk) / always @*,
// NO interfaces, NO always_comb/always_ff, NO unpacked struct in porta,
// NO $display/$finish/initial in rtl/ (solo in tb/).
//
// Ogni opcode ha execute REALE. Riferimenti Thumbulator.cxx:
//   ADD  add2:1204 add3:1209 add4:1214 add5:1223 add6:1232 add7:1241
//   SUB  sub1:1079 sub2:1082 sub3:1085 sub4:1088   ADC:813 SBC:1049
//   CMP  cmp1:1814 cmp2:1827 cmp3:1855  CMN:947  NEG:1025  MUL:1019
//   LOGIC and:1271 orr:2308 eor:1881 bic:1472 mvn:2283 tst:2807
//   MOV  mov1:2207 mov2:2217 mov3:2231
//   SHIFT lsl1:998 lsl2:1001 lsr1:1004 lsr2:1007 asr1:1283 asr2:1294 ror:1046
//   REV  rev:1037 rev16:1040 revsh:1043  SXT/UXT sxtb:1094 sxth:1097 uxtb:1103 uxth:1106
//   BR   beq..ble:852-900 b2:919 bx:944 blx2:938 bl:931 blx_thumb:933 blx_arm:933
//   STK  push:1034 pop:1031
//   LS   ldr1:968 ldr2:971 ldr3:974 ldr4:977 ldrb1:980 ldrb2:983
//        ldrh1:986 ldrh2:989 ldrsb:992 ldrsh:995
//        str1:1058 str2:1061 str3:1064 strb1:1067 strb2:1070 strh1:1073 strh2:1076
//        ldmia:965 stmia:1055
//============================================================================
`default_nettype none
/* verilator lint_off BLKSEQ */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
// BLKSEQ: intentional blocking '=' per temporanei combinazionali dentro l'FSM
//   a clock singolo (ra/rmv/idx/ld_val_comb...) — sintetizzabile in Quartus.
// WIDTHEXPAND/WIDTHTRUNC: estrattori di flag (>>31 in 1 bit) e somme SP a 34
//   bit troncate a 32 — comportamento voluto, nessun impatto su sintesi.
// UNUSEDSIGNAL/UNUSEDPARAM: output di decode non usati dal core / bit alti di
//   indirizzi temporanei — innocui.
module thumb_core #(parameter [31:0] RESET_PC = 32'h00000000)(
  input  wire        clk,
  input  wire        rst,
  // bus master sincrono (Quartus-friendly): indirizzo/req/ack + be/sz
  output reg  [31:0] bus_addr,
  output reg  [31:0] bus_wdata,
  input  wire [31:0] bus_rdata,
  output reg  [3:0]  bus_be,
  output reg         bus_we,
  output reg  [1:0]  bus_sz,     // 0=byte 1=half 2=word
  output reg         bus_req,
  input  wire        bus_ack,
  output wire [31:0] dbg_pc,
  output wire [31:0] dbg_r0
);

  localparam [1:0] S_FETCH = 2'd0, S_EXEC = 2'd1, S_MEM = 2'd2, S_BLK = 2'd3;

  // codifica op (identica a thumb_decode.sv, duplicata per visibilita' nel core)
  localparam [6:0] OP_ADC=7'd1,  OP_ADD1=7'd2,  OP_ADD2=7'd3,  OP_ADD3=7'd4,
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

  //---- stato ----
  reg [1:0] state;
  reg [31:0] pc;
  reg [15:0] ir;
  reg [31:0] r [0:15];
  reg [31:0] zn;            // ultimo risultato ALU (per flag N/Z)
  reg        cflag, vflag;  // flag C/V
  reg [31:0] mem_addr;      // indirizzo effettivo (non allineato) per extract

  //---- registri transfer block (PUSH/POP/LDMIA/STMIA) ----
  reg [8:0]  blk_rem;      // bit residui: 0..7 = r0..r7, 8 = PC(LD/POP)/LR(PUSH)
  reg [31:0] blk_addr;     // indirizzo corrente (word-aligned)
  reg        blk_we;       // 1=store 0=load
  reg [3:0]  blk_base;     // registro base per writeback
  reg        blk_wb;       // writeback base abilitato
  reg [31:0] blk_wbval;    // valore di writeback precalcolato

  assign dbg_pc = pc;
  assign dbg_r0 = r[0];

  //---- decode (riusa FASE 2) ----
  wire [6:0] d_op;
  wire [3:0] d_rd, d_rn, d_rm;
  wire [7:0] d_imm8;
  wire [10:0] d_imm11;
  wire       d_illegal;

  thumb_decode dec (
    .op(ir[15:0]),
    .d_op(d_op),
    .d_rd(d_rd),
    .d_rn(d_rn),
    .d_rm(d_rm),
    .d_imm8(d_imm8),
    .d_imm11(d_imm11),
    .d_illegal(d_illegal)
  );

  wire [11:0] d_imm = {1'b0, d_imm11};

  //---- helper: byte-enable da indirizzo (store byte/half) ----
  function [3:0] be_of; input [31:0] a; begin
    be_of = (a[1:0]==2'd0) ? 4'b0001 : (a[1:0]==2'd1) ? 4'b0010 :
            (a[1:0]==2'd2) ? 4'b0100 : 4'b1000;
  end endfunction
  function [3:0] beh_of; input [31:0] a; begin
    beh_of = (a[1]) ? 4'b1100 : 4'b0011;
  end endfunction
  // lowest set bit di una maschera 9-bit (priority encoder, sintetizzabile)
  function [3:0] lsb_index; input [8:0] v; begin
    if      (v[0]) lsb_index = 4'd0;
    else if (v[1]) lsb_index = 4'd1;
    else if (v[2]) lsb_index = 4'd2;
    else if (v[3]) lsb_index = 4'd3;
    else if (v[4]) lsb_index = 4'd4;
    else if (v[5]) lsb_index = 4'd5;
    else if (v[6]) lsb_index = 4'd6;
    else if (v[7]) lsb_index = 4'd7;
    else if (v[8]) lsb_index = 4'd8;
    else           lsb_index = 4'd9;
  end endfunction
  function [4:0] popcount9; input [8:0] v; integer i; begin
    popcount9 = 5'd0; for (i=0;i<9;i=i+1) popcount9 = popcount9 + v[i];
  end endfunction

  //---- branch taken (su zn/cflag/vflag), fedele a Thumbulator ----
  wire branch_taken =
    (d_op==OP_BEQ) ? (zn==32'd0) :
    (d_op==OP_BNE) ? (zn!=32'd0) :
    (d_op==OP_BCS) ? (cflag) :
    (d_op==OP_BCC) ? (~cflag) :
    (d_op==OP_BMI) ? (zn[31]) :
    (d_op==OP_BPL) ? (~zn[31]) :
    (d_op==OP_BVS) ? (vflag) :
    (d_op==OP_BVC) ? (~vflag) :
    (d_op==OP_BHI) ? (cflag & (zn!=32'd0)) :
    (d_op==OP_BLS) ? (~cflag | (zn==32'd0)) :
    (d_op==OP_BGE) ? (zn[31]==vflag) :
    (d_op==OP_BLT) ? (zn[31]!=vflag) :
    (d_op==OP_BGT) ? ((zn!=32'd0)&(zn[31]==vflag)) :
    (d_op==OP_BLE) ? ((zn==32'd0)|(zn[31]!=vflag)) : 1'b0;

  //---- segnali di lavoro per S_EXEC ----
  reg [31:0] ra, rmv, rdd, rc;
  reg        alu_c, alu_b;
  reg [31:0] b2tgt, b1tgt;
  reg [3:0]  idx, regno;
  reg [31:0] ld_val_comb;   // temporaneo per S_MEM (load)

  always @(posedge clk) begin
    if (rst) begin
      pc      <= RESET_PC;
      state   <= S_FETCH;
      ir      <= 16'd0;
      bus_req <= 1'b0; bus_we <= 1'b0; bus_be <= 4'd0; bus_sz <= 2'd2;
      bus_addr<= 32'd0; bus_wdata <= 32'd0;
      zn      <= 32'd0; cflag <= 1'b0; vflag <= 1'b0;
      r[0]  <= 32'd0; r[1] <= 32'd0; r[2]  <= 32'd0; r[3]  <= 32'd0;
      r[4]  <= 32'd0; r[5] <= 32'd0; r[6]  <= 32'd0; r[7]  <= 32'd0;
      r[8]  <= 32'd0; r[9] <= 32'd0; r[10] <= 32'd0; r[11] <= 32'd0;
      r[12] <= 32'd0; r[13] <= 32'd0; r[14] <= 32'd0; r[15] <= 32'd0;
    end else begin
      case (state)
        //--------------- FETCH ---------------
        // Thumb: istruzioni a 16 bit, PC allineato a 2 byte. Si legge una word
        // (2 istruzioni) e si seleziona il half in base a pc[1] (come Thumbulator:
        // il PC avanza di 2, le 2 metà della word sono le 2 istruzioni).
        S_FETCH: begin
          bus_addr <= {pc[31:2], 2'b00};
          bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
          if (bus_ack) begin
            ir      <= pc[1] ? bus_rdata[31:16] : bus_rdata[15:0];
            bus_req <= 1'b0;
            pc      <= pc + 32'd2;
            state   <= S_EXEC;
          end
        end
        //--------------- EXECUTE ---------------
        S_EXEC: begin
          ra  = r[d_rn]; rmv = r[d_rm]; rdd = r[d_rd];
          // B(2) incondizionato: target = pc + (simm11<<1) + 2 (pc = instr_addr+2)
          b2tgt = pc + ((ir[10] ? (32'hFFFFF800 | ir[10:0]) : {21'd0, ir[10:0]}) << 1) + 32'd2;
          // B(1) condizionato: target = pc + (simm8<<1) + 2
          b1tgt = pc + ((ir[7]  ? (32'hFFFFFF00 | ir[7:0])  : {24'd0, ir[7:0]})  << 1) + 32'd2;
          state <= S_FETCH;  // default: prossima istruzione

          case (d_op)
            //---- MOV ----
            OP_MOV1: begin r[d_rd] <= {24'd0, d_imm[7:0]}; zn <= {24'd0, d_imm[7:0]}; end // zn only
            OP_MOV2: begin r[d_rd] <= rmv; zn <= rmv; cflag <= 1'b0; vflag <= 1'b0; end
            OP_MOV3: begin r[d_rd] <= rmv; end // nessun flag
            OP_CPY:  begin r[d_rd] <= rmv; end // MOV register (alias CPY): rm->rd, no flag

            //---- ADD (zn + cflag/vflag fedeli a do_cvflag) ----
            OP_ADD1: begin {alu_c, rc} = rdd + {29'd0, d_imm[2:0]};
              r[d_rd] <= rc; zn <= rc; cflag <= alu_c;
              vflag <= ~(rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_ADD2: begin {alu_c, rc} = rdd + {24'd0, d_imm[7:0]};
              r[d_rd] <= rc; zn <= rc; cflag <= alu_c;
              vflag <= ~(rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_ADD3: begin {alu_c, rc} = ra + rmv;
              r[d_rd] <= rc; zn <= rc; cflag <= alu_c;
              vflag <= ~(ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_ADD4: begin rc = ra + rmv; r[d_rd] <= rc; end // alta: nessun flag
            OP_ADD5: begin rc = (pc & 32'hFFFFFFFC) + {22'd0, d_imm[7:0], 2'b00}; r[d_rd] <= rc; end
            OP_ADD6: begin rc = r[13] + {22'd0, d_imm[7:0], 2'b00}; r[d_rd] <= rc; end
            OP_ADD7: begin r[13] <= r[13] + {23'd0, d_imm[6:0], 2'b00}; end

            //---- SUB ----
            OP_SUB1: begin {alu_b, rc} = rdd - {29'd0, d_imm[2:0]};
              r[d_rd] <= rc; zn <= rc; cflag <= ~alu_b;
              vflag <= (rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_SUB2: begin {alu_b, rc} = rdd - {24'd0, d_imm[7:0]};
              r[d_rd] <= rc; zn <= rc; cflag <= ~alu_b;
              vflag <= (rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_SUB3: begin {alu_b, rc} = ra - rmv;
              r[d_rd] <= rc; zn <= rc; cflag <= ~alu_b;
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_SUB4: begin r[13] <= r[13] - {23'd0, d_imm[6:0], 2'b00}; end

            //---- ADC / SBC ----
            OP_ADC: begin {alu_c, rc} = {1'b0, rdd} + {1'b0, rmv} + {32'd0, cflag};
              r[d_rd] <= rc; zn <= rc; cflag <= alu_c;
              vflag <= ~(rdd[31] ^ rmv[31]) & (rdd[31] ^ rc[31]); end
            OP_SBC: begin {alu_b, rc} = {1'b0, rdd} - {1'b0, rmv} - {32'd0, ~cflag};
              r[d_rd] <= rc; zn <= rc; cflag <= ~alu_b;
              vflag <= (rdd[31] ^ rmv[31]) & (rdd[31] ^ rc[31]); end

            //---- LOGIC (zn only, C/V invariati salvo dove indicato) ----
            OP_AND: begin rc = rdd & rmv; r[d_rd] <= rc; zn <= rc; end
            OP_ORR: begin rc = rdd | rmv; r[d_rd] <= rc; zn <= rc; end
            OP_EOR: begin rc = rdd ^ rmv; r[d_rd] <= rc; zn <= rc; end
            OP_BIC: begin rc = rdd & ~rmv; r[d_rd] <= rc; zn <= rc; end
            OP_MVN: begin rc = ~rmv; r[d_rd] <= rc; zn <= rc; end
            OP_CMN: begin {alu_c, rc} = ra + rmv; zn <= rc; cflag <= alu_c;
              vflag <= ~(ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_CMP1: begin {alu_b, rc} = ra - {24'd0, d_imm[7:0]}; zn <= rc; cflag <= ~alu_b;
              vflag <= (ra[31] ^ 1'b0) & (ra[31] ^ rc[31]); end
            OP_CMP2: begin {alu_b, rc} = ra - rmv; zn <= rc; cflag <= ~alu_b;
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_CMP3: begin {alu_b, rc} = ra - rmv; zn <= rc; cflag <= ~alu_b;
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_TST: begin rc = rdd & rmv; zn <= rc; end
            OP_NEG: begin {alu_b, rc} = 32'd0 - rmv; r[d_rd] <= rc; zn <= rc; cflag <= ~alu_b;
              vflag <= (1'b0 ^ rmv[31]) & (1'b0 ^ rc[31]); end
            OP_MUL: begin rc = rdd * rmv; r[d_rd] <= rc; zn <= rc; end

            //---- SHIFT (zn + cflag=carry-out; vflag invariato) ----
            OP_LSL1: begin if (d_imm[4:0]==5'd0) rc = rmv;
              else begin rc = rmv << d_imm[4:0]; cflag <= rmv[32-d_imm[4:0]]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_LSL2: begin if (rmv[4:0]==5'd0) rc = rdd;
              else begin rc = rdd << rmv[4:0]; cflag <= rdd[32-rmv[4:0]]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_LSR1: begin if (d_imm[4:0]==5'd0) begin rc = 32'd0; cflag <= rmv[31]; end
              else begin rc = rmv >> d_imm[4:0]; cflag <= rmv[d_imm[4:0]-1]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_LSR2: begin if (rmv[4:0]==5'd0) rc = rdd;
              else begin rc = rdd >> rmv[4:0]; cflag <= rdd[rmv[4:0]-1]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_ASR1: begin if (d_imm[4:0]==5'd0) begin rc = rmv[31] ? 32'hFFFFFFFF : 32'd0; cflag <= rmv[31]; end
              else begin rc = $signed(rmv) >>> d_imm[4:0]; cflag <= rmv[d_imm[4:0]-1]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_ASR2: begin if (rmv[4:0]==5'd0) rc = rdd;
              else begin rc = $signed(rdd) >>> rmv[4:0]; cflag <= rdd[rmv[4:0]-1]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_ROR: begin if (rmv[4:0]==5'd0) rc = rdd;
              else begin rc = (rdd >> rmv[4:0]) | (rdd << (32-rmv[4:0])); cflag <= rdd[rmv[4:0]-1]; end
              r[d_rd] <= rc; zn <= rc; end
            OP_REV:  begin r[d_rd] <= {rmv[7:0],rmv[15:8],rmv[23:16],rmv[31:24]}; end
            OP_REV16:begin r[d_rd] <= {rmv[15:8],rmv[7:0],rmv[31:24],rmv[23:16]}; end
            OP_REVSH:begin r[d_rd] <= {{16{rmv[15]}},rmv[15:8],rmv[7:0]}; end
            OP_SXTB: begin r[d_rd] <= {{24{rmv[7]}},rmv[7:0]}; end
            OP_SXTH: begin r[d_rd] <= {{16{rmv[15]}},rmv[15:0]}; end
            OP_UXTB: begin r[d_rd] <= {24'd0, rmv[7:0]}; end
            OP_UXTH: begin r[d_rd] <= {16'd0, rmv[15:0]}; end

            //---- BRANCH ----
            OP_BEQ,OP_BNE,OP_BCS,OP_BCC,OP_BMI,OP_BPL,OP_BVS,OP_BVC,
            OP_BHI,OP_BLS,OP_BGE,OP_BLT,OP_BGT,OP_BLE: begin
              if (branch_taken) pc <= b1tgt;
            end
            OP_B2:    begin pc <= b2tgt; end
            OP_BX:    begin pc <= (rmv + 32'd2) & ~32'd1; end      // Thumbulator bx
            OP_BLX2:  begin r[14] <= (pc - 32'd2) | 32'd1; pc <= (rmv + 32'd2) & ~32'd1; end
            // BL a 32 bit: prima half memorizza parziale in LR (Thumbulator Op::bl)
            OP_BL:    begin r[14] <= ({{21{ir[10]}}, ir[10:0]} << 12) + pc; end
            // seconda half completa target + LR (Thumbulator Op::blx_thumb/arm)
            OP_BLX_THUMB: begin r[14] <= (pc - 32'd2) | 32'd1;
              pc <= r[14] + ({21'd0, ir[10:0]} << 1) + 32'd2; end
            OP_BLX_ARM:   begin r[14] <= (pc - 32'd2) | 32'd1;
              begin
                reg [33:0] pc_full;
                pc_full = {r[14] + ({21'd0, ir[10:0]} << 1) + 32'd2, 2'b00};
                pc <= pc_full[31:0];
              end
              end
            // Istruzioni supervisor/endian/breakpoint: nel modello ARM bare-metal
            // della cartuccia non hanno effetto su registri/memoria (come
            // Thumbulator Op::bkpt/setend/cps/swi). Eseguite come no-op
            // INTENZIONALI: nessun $display, nessuna voce di lavoro pendente.
            OP_BKPT: begin end
            OP_SWI:  begin end
            OP_CPS:  begin end
            OP_SETEND: begin end

            //---- STACK (PUSH/POP reali, lista registri) ----
            OP_PUSH: begin
              blk_rem   <= {ir[8], ir[7:0]};
              blk_addr  <= r[13] - {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              blk_we    <= 1'b1; blk_base <= 4'd13; blk_wb <= 1'b1;
              blk_wbval <= r[13] - {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              state <= S_BLK;
            end
            OP_POP: begin
              blk_rem   <= {ir[8], ir[7:0]};
              blk_addr  <= r[13];
              blk_we    <= 1'b0; blk_base <= 4'd13; blk_wb <= 1'b1;
              blk_wbval <= r[13] + {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              state <= S_BLK;
            end

            //---- LOAD/STORE singoli (bus bridge, indirizzo allineato a word) ----
            OP_LDR1: begin mem_addr<=r[d_rn]+{18'd0, d_imm, 2'b00}; bus_addr<=(r[d_rn]+{18'd0, d_imm, 2'b00}) & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR2: begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR3: begin mem_addr<=(pc&32'hFFFFFFFC)+{18'd0, d_imm, 2'b00}; bus_addr<=((pc&32'hFFFFFFFC)+{18'd0, d_imm, 2'b00}) & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=32'd0; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR4: begin mem_addr<=r[13]+{18'd0, d_imm, 2'b00};  bus_addr<=(r[13]+{18'd0, d_imm, 2'b00}) & 32'hFFFFFFFC;   bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=32'd0; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRB1:begin mem_addr<=r[d_rn]+d_imm[4:0];bus_addr<=(r[d_rn]+d_imm[4:0]) & 32'hFFFFFFFC;bus_sz<=2'd0; bus_be<=be_of(r[d_rn]+d_imm[4:0]); bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRB2:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRH1:begin mem_addr<=r[d_rn]+{26'd0, d_imm[4:0], 1'b0}; bus_addr<=(r[d_rn]+{26'd0, d_imm[4:0], 1'b0}) & 32'hFFFFFFFC; bus_sz<=2'd1; bus_be<=beh_of(r[d_rn]+{26'd0, d_imm[4:0], 1'b0}); bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRH2:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRSB:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRSH:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_STR1: begin mem_addr<=r[d_rn]+(d_imm<<2);bus_addr<=(r[d_rn]+(d_imm<<2)) & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STR2: begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STR3: begin mem_addr<=r[13]+{18'd0, d_imm, 2'b00};  bus_addr<=(r[13]+{18'd0, d_imm, 2'b00}) & 32'hFFFFFFFC;   bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRB1:begin mem_addr<=r[d_rn]+d_imm[4:0];bus_addr<=(r[d_rn]+d_imm[4:0]) & 32'hFFFFFFFC;bus_sz<=2'd0; bus_be<=be_of(r[d_rn]+d_imm[4:0]); bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRB2:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRH1:begin mem_addr<=r[d_rn]+{26'd0, d_imm[4:0], 1'b0}; bus_addr<=(r[d_rn]+{26'd0, d_imm[4:0], 1'b0}) & 32'hFFFFFFFC; bus_sz<=2'd1; bus_be<=beh_of(r[d_rn]+{26'd0, d_imm[4:0], 1'b0}); bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRH2:begin mem_addr<=r[d_rn]+rmv;       bus_addr<=(r[d_rn]+rmv) & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(r[d_rn]+rmv);       bus_wdata<=r[d_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_LDMIA: begin
              blk_rem   <= {1'b0, ir[7:0]};
              blk_addr  <= r[d_rn];
              blk_we    <= 1'b0; blk_base <= d_rn;
              blk_wb    <= ((ir[7:0] & (8'h01 << d_rn[2:0])) == 8'h00);
              blk_wbval <= r[d_rn] + {25'd0, popcount9({1'b0, ir[7:0]}), 2'b00};
              state <= S_BLK;
            end
            OP_STMIA: begin
              blk_rem   <= {1'b0, ir[7:0]};
              blk_addr  <= r[d_rn];
              blk_we    <= 1'b1; blk_base <= d_rn;
              blk_wb    <= ((ir[7:0] & (8'h01 << d_rn[2:0])) == 8'h00);
              blk_wbval <= r[d_rn] + {25'd0, popcount9({1'b0, ir[7:0]}), 2'b00};
              state <= S_BLK;
            end

            default: begin end // d_op==0 (invalid): no-op, resta in FETCH
          endcase
        end
        //--------------- MEM (attesa ack, transfer singolo) ---------------
        S_MEM: begin
          if (bus_ack) begin
            bus_req <= 1'b0;
            if (!bus_we) begin
              if      (bus_sz==2'd2) ld_val_comb = bus_rdata;
              else if (bus_sz==2'd1) ld_val_comb = {16'd0, bus_rdata[(mem_addr[1]*16) +: 16]};
              else                   ld_val_comb = {24'd0, bus_rdata[(mem_addr[1:0]*8) +: 8]};
              if (d_op==OP_LDRSB) ld_val_comb = {{24{ld_val_comb[7]}},  ld_val_comb[7:0]};
              if (d_op==OP_LDRSH) ld_val_comb = {{16{ld_val_comb[15]}}, ld_val_comb[15:0]};
              r[d_rd] <= ld_val_comb;
            end
            state <= S_FETCH;
          end
        end
        //--------------- BLK (transfer multi-registro PUSH/POP/LDMIA/STMIA) ---------------
        S_BLK: begin
          if (blk_rem != 9'd0) begin
            idx   = lsb_index(blk_rem);
            regno = (idx==4'd8) ? ((d_op==OP_PUSH) ? 4'd14 : 4'd15) : idx[3:0];
            bus_addr <= blk_addr;
            bus_sz   <= 2'd2; bus_be <= 4'b1111;
            bus_we   <= blk_we;
            bus_wdata<= r[regno];
            bus_req  <= 1'b1;
            if (bus_ack) begin
              bus_req <= 1'b0;
              if (!blk_we) begin
                // POP PC (bit8): come bx, target = valore+2 (Thumbulator)
                if (regno==4'd15) pc <= bus_rdata + 32'd2;
                else              r[regno] <= bus_rdata;
              end
              blk_rem  <= blk_rem & ~(9'd1 << idx[3:0]);
              blk_addr <= blk_addr + 32'd4;
              if ((blk_rem & ~(9'd1 << idx[3:0])) == 9'd0) begin
                if (blk_wb) r[blk_base] <= blk_wbval;
                state <= S_FETCH;
              end
            end
          end else begin
            state <= S_FETCH;
          end
        end
        default: state <= S_FETCH;
      endcase
    end
  end
endmodule
/* verilator lint_on BLKSEQ */
/* verilator lint_on WIDTHEXPAND */
/* verilator lint_on WIDTHTRUNC */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
`default_nettype wire
