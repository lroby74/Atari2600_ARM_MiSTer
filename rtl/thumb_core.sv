//============================================================================
// thumb_core.sv  —  core Thumb: fetch + decode(riusato) + execute + bus bridge
// (FASE 3). Basato su stella-emu/stella  src/emucore/Thumbulator.cxx
// (public domain / GPL, David Welch / F. Quimby). Nessun firmware proprietario.
// Stile SystemVerilog-2005, "Sorgelig": always @(posedge clk) / always @*,
// NO interfaces, NO always_comb/always_ff, NO unpacked struct in porta,
// NO $display/$finish/initial: qui dentro c'e' solo hardware sintetizzabile.
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
module thumb_core (
  input  wire        clk,
  input  wire        rst,
  // run control: the core sits halted until `start` pulses; a run ends (and
  // `halted` rises) when a BX/BLX reaches an even (ARM-mode) address that is
  // not one of the music-callback trap addresses - Thumbulator semantics,
  // including the driver main() returning to LR = cBase.
  input  wire        start,
  input  wire [31:0] start_pc,
  input  wire [31:0] start_sp,
  input  wire [31:0] start_lr,
  output reg         halted,
  // music-callback trap: Thumbulator pc values (BX instr address + 4)
  input  wire [31:0] cb_addr0,   // _SetNote(r2,r3)
  input  wire [31:0] cb_addr1,   // _ResetWave(r2)
  input  wire [31:0] cb_addr2,   // _GetWavePtr(r2) -> r2
  input  wire [31:0] cb_addr3,   // _SetWaveSize(r2,r3)
  output reg         cb_req,
  output reg  [1:0]  cb_id,
  output wire [31:0] cb_v1,
  output wire [31:0] cb_v2,
  input  wire        cb_ack,
  input  wire [31:0] cb_ret,
  // bus master sincrono (Quartus-friendly): indirizzo/req/ack + be/sz
  output reg  [31:0] bus_addr,
  output reg  [31:0] bus_wdata,
  input  wire [31:0] bus_rdata,
  output reg  [3:0]  bus_be,
  output reg         bus_we,
  output reg  [1:0]  bus_sz,     // 0=byte 1=half 2=word
  output reg         bus_req,
  input  wire        bus_ack,
  // word successiva a bus_addr, servita gratis dalla ROM sdoppiata; valida
  // solo quando bus_rdata_next_ok e' alto (regione ROM del bridge CDF).
  input  wire [31:0] bus_rdata_next,
  input  wire        bus_rdata_next_ok,
  // STADIO 2 - INDIRIZZO ANTICIPATO. `bus_addr_pre` e' l'indirizzo efficace
  // dell'accesso a memoria che sta per essere emesso, disponibile gia' durante
  // S_EXEC (combinatorio), un ciclo PRIMA di `bus_addr`. Il bridge lo presenta
  // alla porta A della M10K, cosi' il dato e' valido al primo ciclo di S_MEM
  // invece che al secondo.
  output wire [31:0] bus_addr_pre,
  // PASSO 15: lo stesso indirizzo PIU' QUATTRO, da un sommatore
  // PARALLELO (stessi operandi), non in cascata. Serve alla ROM per
  // la rotazione dei banchi del fetch a 64 bit: li' il `+1` diventa
  // un mux 2:1 invece di un secondo sommatore in serie.
  output wire [31:0] bus_addr_pre_nx,
  output wire        bus_req_pre,
  output wire [31:0] dbg_pc,
  output wire [31:0] dbg_r0
);

  localparam [2:0] S_FETCH = 3'd0, S_EXEC = 3'd1, S_MEM = 3'd2, S_BLK = 3'd3, S_CB = 3'd4;
  // PASSO 4: secondo ciclo della MUL, vedi il blocco `mul_a`/`mul_b`.
  localparam [2:0] S_MUL = 3'd5;
  // PASSO 8: secondo ciclo degli scorrimenti a registro.
  localparam [2:0] S_SHIFT = 3'd6;

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
  reg [2:0] state;
  // fetch cache a 8 entry direct-mapped su addr[4:2] (Step 6 speed): una word
  // da 32 bit contiene due istruzioni Thumb; 8 entry coprono loop fino a 16
  // istruzioni distribuite su 8 word consecutive senza collisioni (il loop
  // fill/memset di Draconian usa 4 word distinte per iterazione e con una
  // cache a 1 o 2 entry andava in thrash perenne: 66% miss misurato).
  // Letture asincrone multiple + scritture indicizzate: Quartus la mappa su
  // registri (stesso pattern del register file r[0:15]), NON inferisce RAM.
  // 3 ago 2026: allargata da 8 a 16 SET (indice a[6:2] invece di a[4:2]),
  // sempre 2 vie -> 64 entry. Con 8 set la cache copriva appena 32 byte di
  // codice per via: bastava al loop stretto di Draconian (91,8% di hit) ma i
  // CDFJ+ la distruggevano - Elevator Agent 16,2% di hit, Tutankham 0,3%, ed
  // entrambi spendevano il 30% dei cicli ARM in S_FETCH contro il 4% di
  // Draconian. Elevator Agent sforava la VBlank 7 run su 11 per questo.
  // FASE-1 PROVA: cache 2-way set-associative, ora 16 set x 2 vie = 32 entry.
  // Indicizzazione invariata su addr[4:2] (stessi 8 set), ma ogni set ha
  // due vie -> i conflict miss su indirizzi con stesso indice e tag diverso
  // spariscono. Politica di rimpiazzo: LRU a 1 bit per set (con 2 vie e'
  // esatta, non un'approssimazione).
  // indirizzo della word SUCCESSIVA a quella puntata da pc (fetch a 64 bit)
  wire [31:0] pc_nx = {pc[31:2], 2'b00} + 32'd4;
  // PASSO 20: 32 SET x 2 vie = 64 entry (indice a[6:2] invece di a[6:2]).
  // Raddoppia il codice coperto per via: serve ai CDFJ+, che sui 16 set
  // hanno ancora un tasso di successo molto piu' basso di Draconian.
  reg [31:0] fetch_word [0:63];   // {via, set}: indice = {w, set[4:0]}
  reg [29:0] fetch_tag  [0:63];
  reg [63:0] fetch_valid;
  reg [31:0] fetch_lru;           // per set: quale via e' la LRU (da rimpiazzare)

  // hit su una qualunque delle due vie del set.
  // Gli indici sono variabili a 4 bit DICHIARATE (i0 = via 0, i1 = via 1)
  // invece di concatenazioni inline: gli array sono [0:15] e 4 bit bastano,
  // ma cosi' l'analizzatore Quartus non deve dedurre la larghezza da
  // un'espressione (warning 10027). Sono assegnate incondizionatamente,
  // quindi non possono a loro volta generare un 10776.
  function fc_hit; input [31:0] a; reg [5:0] i0, i1; begin
    i0 = {1'b0, a[6:2]};
    i1 = {1'b1, a[6:2]};
    fc_hit = (fetch_valid[i0] && fetch_tag[i0] == a[31:2]) ||
             (fetch_valid[i1] && fetch_tag[i1] == a[31:2]);
  end endfunction
  // via che contiene l'indirizzo (valida solo se fc_hit)
  function fc_way; input [31:0] a; reg [5:0] i0; begin
    i0 = {1'b0, a[6:2]};
    fc_way = (fetch_valid[i0] && fetch_tag[i0] == a[31:2]) ? 1'b0 : 1'b1;
  end endfunction
  // word letta dal set/via corretti.
  // PASSO 19: le DUE VIE si leggono IN PARALLELO. Prima l'indice era a 5 bit
  // e il suo MSB era `fc_way(a)`, cioe' il risultato del confronto dei tag: il
  // mux 32:1 aspettava la comparazione, ed erano 2,90 ns in serie sul cammino
  // critico. Ora i due mux 16:1 (indicizzati dal solo set, pronto subito)
  // corrono INSIEME al confronto, e in serie resta un mux 2:1.
  function [31:0] fc_word; input [31:0] a; reg [31:0] w0, w1; begin
    w0 = fetch_word[{1'b0, a[6:2]}];
    w1 = fetch_word[{1'b1, a[6:2]}];
    fc_word = fc_way(a) ? w1 : w0;
  end endfunction
  // half-word (istruzione Thumb) selezionata da a[1].
  // NB: Verilog-2005 non ammette bit-select sul ritorno di una function
  // (`fc_word(a)[31:16]` e' illegale) -> serve un temporaneo interno.
  // PASSO 19: anche la scelta della meta' si fa PRIMA di quella della via.
  // `a[1]` e' un bit di indirizzo, disponibile subito; `fc_way` arriva dal
  // confronto dei tag. Facendo prima la mezza parola su entrambe le vie, in
  // serie al confronto resta un mux 2:1 a 16 bit invece che a 32.
  function [15:0] fc_ir; input [31:0] a;
    reg [31:0] w0, w1; reg [15:0] h0, h1; begin
    w0 = fetch_word[{1'b0, a[6:2]}];
    w1 = fetch_word[{1'b1, a[6:2]}];
    h0 = a[1] ? w0[31:16] : w0[15:0];
    h1 = a[1] ? w1[31:16] : w1[15:0];
    fc_ir = fc_way(a) ? h1 : h0;
  end endfunction
  reg [31:0] pc;
  reg [15:0] ir;
  reg [31:0] r [0:15];
  // PASSO 26b: copia integrale del banco per il PERCORSO DI ESECUZIONE.
  // Stessi valori, stessi istanti di scrittura. Esiste perche' i lettori
  // dell'indirizzo effettivo e quelli dell'esecuzione stanno in due zone
  // diverse del chip: con un banco solo il piazzatore paga il compromesso
  // su entrambi (2,06 ns di instradamento sul cammino peggiore).
  reg [31:0] rE [0:15];
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
  assign cb_v1 = r[2];
  assign cb_v2 = r[3];

  //---- PASSO 11: DECODE REGISTRATO ALLA SORGENTE ----
  //
  // I campi decodificati NON escono piu' da un decoder combinatorio pilotato da
  // `ir`: sono REGISTRI, caricati insieme a `ir` dal decoder della sorgente
  // giusta. Il decoder esce cosi' dal cammino `ir -> r[]`, che il timing report
  // misurava a 2,13 ns su 15,51.
  //
  // REGOLA DA RISPETTARE SEMPRE: ogni punto che assegna `ir <= X` DEVE
  // assegnare anche `dr_* <= <decode di X>`. Sono 10 punti, elencati sotto.
  // Se ne sfugge uno, l'istruzione viene eseguita con la decodifica di quella
  // PRECEDENTE - e il guasto e' silenzioso.
  reg [6:0]  dr_op;
  reg [3:0]  dr_rd, dr_rn, dr_rm;
  // 17 ago 2026 - tolta la catena dell'immediato a 8 bit (dr_imm8/pd_imm8/
  // d_imm8): veniva calcolata, registrata e non la leggeva NESSUNO. Il campo
  // immediato in uso e' dr_imm11 -> dr_imm. Erano warning 10036.
  reg [10:0] dr_imm11;
  wire [11:0] dr_imm = {1'b0, dr_imm11};
  // PASSO 11c: `dr_illegal` NON si registra, si DERIVA dal registro `dr_op`.
  // Nel decoder `d_illegal` e' `(d_op == OP_UNDEF)` calcolato DOPO tutto il
  // `casez`: e' l'uscita piu' profonda, e infatti tutti i cammini peggiori
  // misurati finivano proprio li'. Derivandola a valle del registro, quel
  // confronto esce dal cammino temporizzato. Serve solo a `fast_ok`.
  // (`OP_UNDEF` non e' fra i localparam di questo file: la lista del core parte
  //  da OP_ADC=1. Il valore e' 0, come in thumb_decode.sv.)
  wire       dr_illegal = (dr_op == 7'd0);
  // PASSO 11b: bersagli dei salti, anch'essi registrati al carico di `ir`
  // PASSO 33: UN SOLO bersaglio. B(1) e B(2) sono mutuamente esclusivi,
  // e quale dei due serva si sa gia' al carico dell'istruzione.
  reg [31:0] brt;

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
    (dr_op==OP_BEQ) ? (zn==32'd0) :
    (dr_op==OP_BNE) ? (zn!=32'd0) :
    (dr_op==OP_BCS) ? (cflag) :
    (dr_op==OP_BCC) ? (~cflag) :
    (dr_op==OP_BMI) ? (zn[31]) :
    (dr_op==OP_BPL) ? (~zn[31]) :
    (dr_op==OP_BVS) ? (vflag) :
    (dr_op==OP_BVC) ? (~vflag) :
    (dr_op==OP_BHI) ? (cflag & (zn!=32'd0)) :
    (dr_op==OP_BLS) ? (~cflag | (zn==32'd0)) :
    (dr_op==OP_BGE) ? (zn[31]==vflag) :
    (dr_op==OP_BLT) ? (zn[31]!=vflag) :
    (dr_op==OP_BGT) ? ((zn!=32'd0)&(zn[31]==vflag)) :
    (dr_op==OP_BLE) ? ((zn==32'd0)|(zn[31]!=vflag)) : 1'b0;

  //---- PASSO 5: SALTO PRESO, RICAVATO DIRETTAMENTE DA `ir` ----
  //
  // Stessa informazione di `cond_br && branch_taken` e `dr_op == OP_B2`, ma
  // senza passare dal decoder: sul cammino critico il decoder valeva ~2,9 ns
  // prima ancora di arrivare al bersaglio del salto.
  //
  // Corrispondenza verificata su thumb_decode.sv:185-191 -
  //   1101 cccc iiiiiiii  con cccc <= 13  ->  OP_BEQ + cccc  (cccc 14 = UNDEF,
  //                                                           15 = SWI)
  //   11100 iiiiiiiiiii                    ->  OP_B2
  wire       br_is_b1_raw = (ir[15:12] == 4'b1101) && (ir[11:8] <= 4'd13);
  wire       br_is_b2_raw = (ir[15:11] == 5'b11100);
  wire [3:0] br_cc_raw    = ir[11:8];
  // stessa tabella di `branch_taken`, indicizzata dal campo condizione invece
  // che da dr_op (dr_op = OP_BEQ + cccc, quindi coincidono uno a uno)
  wire br_cond_raw =
    (br_cc_raw==4'd0)  ? (zn==32'd0) :
    (br_cc_raw==4'd1)  ? (zn!=32'd0) :
    (br_cc_raw==4'd2)  ? (cflag) :
    (br_cc_raw==4'd3)  ? (~cflag) :
    (br_cc_raw==4'd4)  ? (zn[31]) :
    (br_cc_raw==4'd5)  ? (~zn[31]) :
    (br_cc_raw==4'd6)  ? (vflag) :
    (br_cc_raw==4'd7)  ? (~vflag) :
    (br_cc_raw==4'd8)  ? (cflag & (zn!=32'd0)) :
    (br_cc_raw==4'd9)  ? (~cflag | (zn==32'd0)) :
    (br_cc_raw==4'd10) ? (zn[31]==vflag) :
    (br_cc_raw==4'd11) ? (zn[31]!=vflag) :
    (br_cc_raw==4'd12) ? ((zn!=32'd0)&(zn[31]==vflag)) :
    (br_cc_raw==4'd13) ? ((zn==32'd0)|(zn[31]!=vflag)) : 1'b0;
  wire br_fast_take = (br_is_b1_raw && br_cond_raw) || br_is_b2_raw;

  // --- le QUATTRO sorgenti di `ir`, come wire ---
  // (lr_pc era una variabile blocking dentro il blocco sincrono: qui diventa
  //  wire, perche' un modulo non si puo' alimentare da una variabile assegnata
  //  dentro un always. L'espressione e' identica.)
  wire [31:0] lr_pc  = r[14] & ~32'd1;

  wire [15:0] ir_pc  = fc_ir(pc);
  // PASSO 21: le DUE ricerche partono dai REGISTRI b1tgt/b2tgt e corrono
  // insieme; `br_is_b2_raw` sceglie solo alla fine. Sparisce il prefisso
  // `ir -> confronto -> mux` (1,41 ns) davanti alla cache.
  // PASSO 33: UN lookup invece di due; l'indirizzo e' gia' quello giusto.
  wire [15:0] ir_brt = fc_ir(brt);
  // PASSO 27: le quattro sorgenti di `ir` sotto lo stesso indice dei
  // decoder, cosi' entrano nel mux unico invece di avere il loro albero.
  wire [15:0] ir_src [0:3];
  assign ir_src[0] = ir_pc;
  assign ir_src[1] = ir_brt;
  assign ir_src[2] = 16'd0;   // PASSO 32: sorgente eliminata
  assign ir_src[3] = ir_bus;
  wire [15:0] ir_bus = pc[1] ? bus_rdata[31:16] : bus_rdata[15:0];

  //---- PASSO 11b: ANCHE IL BERSAGLIO DEL SALTO E' PRECALCOLATO ----
  //
  // Misurato: col solo passo 11 lo slack PEGGIORA (+1,652 -> +0,581). Il
  // decoder esce dal cammino `ir -> r[]` ma ne nasce uno piu' lungo:
  //   pc -> sommatore b1tgt/b2tgt -> br_tgt -> lookup in cache -> decoder
  // cioe' 14 livelli fino a `dr_illegal`. Il sommatore da solo vale 2,1 ns.
  //
  // Rimedio, stesso principio del passo 11: b1tgt/b2tgt diventano REGISTRI,
  // calcolati quando si carica `ir`. Il bersaglio di un salto dipende solo
  // dall'istruzione e dal suo indirizzo, entrambi noti al momento del carico.
  //   b1tgt = A + 4 + (simm8 <<1)      A = indirizzo dell'istruzione
  //   b2tgt = A + 4 + (simm11<<1)      (`pc` vale A+2, da cui il +4)
  function [31:0] btgt1; input [31:0] a; input [15:0] x; begin
    btgt1 = a + 32'd4 +
            ((x[7]  ? (32'hFFFFFF00 | {24'd0, x[7:0]})  : {24'd0, x[7:0]})  << 1);
  end endfunction
  function [31:0] btgt2; input [31:0] a; input [15:0] x; begin
    btgt2 = a + 32'd4 +
            ((x[10] ? (32'hFFFFF800 | {21'd0, x[10:0]}) : {21'd0, x[10:0]}) << 1);
  end endfunction

  // PASSO 33: quale bersaglio serve si decide QUI, al carico, guardando i
  // bit grezzi dell'istruzione (11100 = B(2)) e non l'uscita del decoder:
  // cosi' il mux corre in PARALLELO al decoder invece che dietro di lui.
  wire [31:0] pre_brt [0:3];
  // indirizzo dell'istruzione, per sorgente (stesso ordine dei decoder)
  wire [31:0] pre_b1 [0:3];
  wire [31:0] pre_b2 [0:3];
  assign pre_b1[0] = btgt1(pc,     ir_pc);
  assign pre_b2[0] = btgt2(pc,     ir_pc);
  assign pre_b1[1] = btgt1(brt, ir_brt);   // PASSO 33
  assign pre_b2[1] = btgt2(brt, ir_brt);
  assign pre_b1[2] = 32'd0;   // PASSO 32
  assign pre_b2[2] = 32'd0;
  assign pre_b1[3] = btgt1(pc,     ir_bus);
  assign pre_b2[3] = btgt2(pc,     ir_bus);
  genvar gb;
  generate for (gb = 0; gb < 4; gb = gb + 1) begin : g_brt
    assign pre_brt[gb] = (ir_src[gb][15:11] == 5'b11100) ? pre_b2[gb]
                                                        : pre_b1[gb];
  end endgenerate

  // --- un decoder per sorgente ---
  wire [6:0] pd_op   [0:3];
  wire [3:0] pd_rd   [0:3], pd_rn [0:3], pd_rm [0:3];
  wire [10:0] pd_imm11[0:3];
  wire       pd_ill  [0:3];

  thumb_decode dec_pc (
    .op(ir_pc),  .d_op(pd_op[0]), .d_rd(pd_rd[0]), .d_rn(pd_rn[0]),
    .d_rm(pd_rm[0]), .d_imm11(pd_imm11[0]),
    .d_illegal(pd_ill[0]));
  // PASSO 33: un decoder solo, sul bersaglio gia' scelto.
  thumb_decode dec_brt (
    .op(ir_brt), .d_op(pd_op[1]), .d_rd(pd_rd[1]), .d_rn(pd_rn[1]),
    .d_rm(pd_rm[1]), .d_imm11(pd_imm11[1]),
    .d_illegal(pd_ill[1]));
  // PASSO 32: il decoder di lr_pc e' sparito con la sua sorgente.
  assign pd_op[2]=7'd0; assign pd_rd[2]=4'd0; assign pd_rn[2]=4'd0;
  assign pd_rm[2]=4'd0; assign pd_imm11[2]=11'd0;
  assign pd_ill[2]=1'b0;
  thumb_decode dec_bus (
    .op(ir_bus), .d_op(pd_op[3]), .d_rd(pd_rd[3]), .d_rn(pd_rn[3]),
    .d_rm(pd_rm[3]), .d_imm11(pd_imm11[3]),
    .d_illegal(pd_ill[3]));

  //---- classificatori per il fast-path fetch+execute (Step 1 speed) ----
  // Un'istruzione e' "fast" se non tocca il bus e non ridirige il PC: in tal
  // caso il fetch della prossima istruzione (se in cache) puo' avvenire nello
  // stesso ciclo di S_EXEC, senza passare da S_FETCH (1 clk/istruzione).
  // I range di opcode sono contigui nei localparam qui sopra.
  // Le forme a REGISTRO ALTO possono avere r15 come destinazione:
  //   add pc,rN   (ADD(4))    mov pc,rN   (MOV(3), include `mov pc,lr`)
  // Sono salti a tutti gli effetti - vanno esclusi dal fast-path, altrimenti
  // l'assegnazione di pc del ramo verrebbe sovrascritta dal fetch anticipato.
  wire wr_pc_hi     = ((dr_op == OP_ADD4) || (dr_op == OP_MOV3) ||
                       (dr_op == OP_CPY)) && (dr_rd == 4'd15);
  // valore che l'ISA Thumb attribuisce a r15 quando lo si LEGGE: indirizzo
  // dell'istruzione + 4, bit0 azzerato (Thumbulator::read_register). Qui `pc`
  // vale gia' instr_addr+2, quindi pc+2. Precalcolato come wire cosi'
  // l'addizionatore sta FUORI dal cammino ir->dr_rn->register-file (che e' il
  // cammino critico misurato del core): sul percorso resta il solo mux.
  wire [31:0] r15_rd = (pc + 32'd2) & 32'hFFFFFFFE;

  //--------------------------------------------------------------------------
  // STADIO 2 - calcolo COMBINATORIO dell'indirizzo efficace di memoria.
  //
  // Stessi operandi dei rami load/store di S_EXEC, ma fuori dal blocco a clock:
  // dentro, `asum` fra due fronti contiene il valore dell'istruzione PRECEDENTE
  // e non servirebbe a niente.
  // Non tocca l'ALU: e' un sommatore dedicato al solo indirizzo, cosi' il
  // cammino verso la memoria non attraversa il mux largo dell'ALU.
  //--------------------------------------------------------------------------
  // PASSO 17: niente mux di r15. In Thumb la base di una load/store a
  // registro viene dai soli registri bassi (il decoder assegna
  // dr_rn = {1'b0, op[5:3]}, dr_rm = {1'b0, op[8:6]}): **al massimo 7**.
  // r15 come base non e' nemmeno rappresentabile, e le forme che lo
  // ammettono (ADD(4)/CMP(3)/MOV(3)) non passano di qui.
  wire [31:0] ea_ra  = r[dr_rn];
  wire [31:0] ea_rmv = r[dr_rm];
  // PASSO 16: scostamento della LDR3 precalcolato. `(pc+2)&~3` e'
  // identicamente `(pc&~3) + (pc[1] ? 4 : 0)` perche' in Thumb `pc` e'
  // sempre pari; cosi' i due sommatori in serie diventano uno solo, e
  // questa somma - fra due valori piccoli e gia' registrati - sta fuori
  // dalla catena lunga.
  wire [31:0] ldr3_off = {18'd0, dr_imm, 2'b00} + (pc[1] ? 32'd4 : 32'd0);
  reg  [31:0] ea;
  reg         ea_is_mem;
  always @* begin
    ea = 32'd0; ea_is_mem = 1'b1;
    case (dr_op)
      OP_LDR2, OP_LDRB2, OP_LDRH2, OP_LDRSB, OP_LDRSH,
      OP_STR2, OP_STRB2, OP_STRH2:  ea = ea_ra + ea_rmv;
      OP_LDR1, OP_STR1:             ea = ea_ra + {18'd0, dr_imm, 2'b00};
      OP_LDRB1, OP_STRB1:           ea = ea_ra + {27'd0, dr_imm[4:0]};
      OP_LDRH1, OP_STRH1:           ea = ea_ra + {26'd0, dr_imm[4:0], 1'b0};
      OP_LDR3:                      ea = (pc & 32'hFFFFFFFC) + ldr3_off;
      OP_LDR4, OP_STR3:             ea = r[13] + {18'd0, dr_imm, 2'b00};
      default:                      ea_is_mem = 1'b0;
    endcase
  end
  // valido solo mentre si sta per emettere l'accesso, cioe' in S_EXEC e senza
  // una richiesta gia' in volo
  assign bus_addr_pre = ea & 32'hFFFFFFFC;
  // PASSO 15: secondo sommatore, IN PARALLELO al primo. Non si scrive
  // `ea + 4` perche' sarebbe in serie: si ripete la stessa selezione
  // di operandi con 4 sommato al termine costante, cosi' le due somme
  // partono insieme dagli stessi registri.
  reg [31:0] ea_nx;
  always @* begin
    ea_nx = 32'd4;
    case (dr_op)
      OP_LDR2, OP_LDRB2, OP_LDRH2, OP_LDRSB, OP_LDRSH,
      OP_STR2, OP_STRB2, OP_STRH2:  ea_nx = ea_ra + ea_rmv + 32'd4;
      OP_LDR1, OP_STR1:             ea_nx = ea_ra + {18'd0, dr_imm, 2'b00} + 32'd4;
      OP_LDRB1, OP_STRB1:           ea_nx = ea_ra + {27'd0, dr_imm[4:0]} + 32'd4;
      OP_LDRH1, OP_STRH1:           ea_nx = ea_ra + {26'd0, dr_imm[4:0], 1'b0} + 32'd4;
      OP_LDR3:                      ea_nx = (pc & 32'hFFFFFFFC) + ldr3_off + 32'd4;
      OP_LDR4, OP_STR3:             ea_nx = r[13] + {18'd0, dr_imm, 2'b00} + 32'd4;
      default:                      ea_nx = 32'd4;
    endcase
  end
  assign bus_addr_pre_nx = ea_nx & 32'hFFFFFFFC;
  assign bus_req_pre  = (state == S_EXEC) && ea_is_mem && !bus_req;
  wire cond_br      = (dr_op >= OP_BEQ) && (dr_op <= OP_BLE);
  wire op_redirects = (cond_br && branch_taken) || (dr_op == OP_B2) ||
                      (dr_op == OP_BX) || (dr_op == OP_BLX2) ||
                      (dr_op == OP_BLX_THUMB) || (dr_op == OP_BLX_ARM) ||
                      wr_pc_hi;
  wire op_uses_bus  = (dr_op == OP_PUSH) || (dr_op == OP_POP) ||
                      (dr_op == OP_LDMIA) || (dr_op == OP_STMIA) ||
                      ((dr_op >= OP_LDR1) && (dr_op <= OP_LDRSH)) ||
                      ((dr_op >= OP_STR1) && (dr_op <= OP_STRH2));
  // PASSO 4: la MUL ora dura due cicli (S_EXEC -> S_MUL), quindi NON puo'
  // stare nel fast path: quello riassegna `state` e `ir` in coda a S_EXEC
  // e cancellerebbe il secondo ciclo, buttando via il prodotto.
  // PASSO 8: anche gli scorrimenti a registro durano due cicli, quindi
  // escono dal fast path per lo stesso motivo della MUL.
  wire op_shift_reg = (dr_op == OP_LSL2) || (dr_op == OP_LSR2) ||
                      (dr_op == OP_ASR2) || (dr_op == OP_ROR);
  wire fast_ok      = !op_redirects && !op_uses_bus && !dr_illegal
                      && (dr_op != OP_MUL) && !op_shift_reg;

  //---- segnali di lavoro per S_EXEC ----
  reg [31:0] ra, rmv, rdd, rc;
  // PASSO 7 - PORTA DI SCRITTURA UNICA del file registri (solo S_EXEC).
  // Le 37 assegnazioni a r[dr_rd] + r[13] + r[14] convergono qui: l'albero di
  // mux per bit di registro passa da 37 ingressi a uno solo. Valeva 4,9 ns su
  // 5 livelli del cammino critico (misura pathdet.tcl, 9 ago 2026).
  // PASSO 27 - porta di CARICO unica dei registri di decodifica, stessa
  // forma della porta di scrittura unica del passo 7/14.
  reg        dr_load;
  reg [1:0]  dr_sel;
  reg        wr_en;
  reg [3:0]  wr_idx;
  reg [31:0] wr_val;
  reg        alu_c, alu_b;
  //---- SOMMATORE CONDIVISO (2 ago 2026) ----------------------------------
  // Prima ognuno dei 18 opcode aritmetici (ADD1-7, SUB1-4, ADC, SBC, CMN,
  // CMP1-3, NEG) istanziava il PROPRIO sommatore a 32 bit, e il mux a ~89 vie
  // del `case (dr_op)` sceglieva fra i risultati. Cammino critico misurato:
  // thumb_core|ir -> r[], con quel mux come parte piu' profonda.
  // Ora gli operandi si selezionano PRIMA (mux stretti) e il sommatore e' UNO.
  //
  // Identita' che rende l'unificazione possibile: in TUTTI e 18 il flag C di
  // ARM e' esattamente il carry-out di  x + (sub ? ~y : y) + cin.
  //   ADD/ADC : {alu_c, rc} = x + y (+c)      -> cflag <= alu_c    = carry
  //   SUB/CMP/NEG : {alu_b, rc} = x - y       -> cflag <= ~alu_b
  //                 e  x - y == x + ~y + 1    -> carry = ~borrow   = carry
  //   SBC     : x - y - ~c == x + ~y + c      -> idem
  // quindi dopo la riscrittura vale sempre e solo  cflag <= asum[32].
  reg [31:0] ax, ay;        // operandi selezionati
  reg        asub, acin;    // sottrazione / carry entrante
  reg [32:0] asum;          // {carry_out, risultato}
  reg [3:0]  idx, regno;
  reg [31:0] ld_val_comb;   // temporaneo per S_MEM (load)
  reg        inv_way;       // via da invalidare (store self-modifying), vedi S_MEM/S_BLK

  //--------------------------------------------------------------------------
  // FIX GLOBALE punto 5 - WATCHDOG DI RUN.
  //
  // PERCHE': le uniche uscite da un run sono `halted <= 1` su BX/BLX2 verso un
  // indirizzo pari (vedi OP_BX/OP_BLX2 in S_EXEC). Un run che si avvita non
  // finisce mai. Aggrava il fatto che un opcode non decodificato e' un no-op
  // silenzioso (OP di default in S_EXEC): si prosegue dentro spazzatura invece
  // di fermarsi. Stella ha lo stesso problema e lo chiude con un guard
  // esplicito: Thumbulator.cxx:181-182, "instructions > 500000" -> eccezione.
  //
  // METRICA: si contano CICLI, non istruzioni - deviazione dichiarata rispetto
  // a Stella, per tre motivi.
  //  1. Su FPGA il vincolo reale e' il TEMPO (il budget di frame), e i cicli lo
  //     misurano direttamente; le istruzioni no, perche' il CPI varia con i
  //     miss di cache.
  //  2. Contare istruzioni richiederebbe di agganciare i punti di retire di una
  //     FSM che ha tre fast-path che rientrano in S_EXEC senza passare da
  //     S_FETCH: modifica con rischio di regressione sulla golden trace, a
  //     fronte di zero beneficio.
  //  3. Un contatore libero nel ramo "in esecuzione" non tocca nient'altro: non
  //     puo' alterare la traccia se non scattando.
  //
  // SOGLIA: contatore a 20 bit, scatto a 2^20-1 = 1 048 575 cicli clk_vid.
  //  - in tempo reale: 18.31 ms a 57.2727 MHz = 1.10 frame NTSC. Un run che non
  //    ha finito entro un frame intero non ha piu' alcuna utilita'.
  //  - equivalenza con Stella: 500 000 istruzioni al CPI misurato 1.99
  //    (cpi*100 = 199 nella regressione Draconian) valgono ~995 000 cicli,
  //    cioe' lo stesso ordine di 2^20.
  //  - NON PUO' SCATTARE in condizioni operative reali. MISURATO sulla
  //    regressione Draconian (200 000 cicli CPU, 21 run ARM): il run piu'
  //    lungo vale 36 830 cicli clk_vid / 21 443 istruzioni -> margine 28.5x
  //    sulla soglia in cicli e 23.3x su quella di Stella in istruzioni.
  //    Il run piu' pesante documentato in Stella (Thumbulator.cxx:192,
  //    _totalCycles = 127148, "VB during Turbo start sequence") vale ~254 000
  //    cicli al CPI 2: margine 4.1x anche in quel caso limite.
  //
  // POSIZIONE: l'assegnazione sta PRIMA del case(state), quindi una fine run
  // legittima nello stesso ciclo (OP_BX/OP_BLX2) sovrascrive il watchdog per
  // last-write-wins. Il watchdog vince solo quando nessuno altro ferma il run.
  //--------------------------------------------------------------------------
  //--------------------------------------------------------------------------
  // PASSO 4 - MOLTIPLICATORE A DUE STADI.
  //
  // Il moltiplicatore era il 72% del cammino critico dell'ARM (8,2 ns su 11,4
  // misurati con pathdet.tcl il 9 ago 2026), e stava in mezzo alla catena
  // `ir -> decoder -> mux registri -> moltiplicatore -> r[]`, cioe' penalizzava
  // la frequenza di TUTTE le istruzioni per colpa di una sola.
  //
  // Registrando gli operandi la catena si spezza in due tratti corti:
  //   ir -> decoder -> mux 8:1 -> mul_a        (nessun moltiplicatore)
  //   mul_a -> moltiplicatore -> r[]           (solo il moltiplicatore)
  //
  // COSTO MISURATO (sonda [opmix], Draconian, 108595 istruzioni): le MUL sono
  // 110, cioe' 1 per mille. Un ciclo in piu' ciascuna = +0,06% di cicli ARM.
  //
  // Gli operandi sono SEMPRE r0-r7: MUL sta nel gruppo 0x4000 del decoder, dove
  // dr_rd e dr_rm valgono {1'b0, op[2:0]} e {1'b0, op[5:3]}. Quindi mux 8:1, non
  // l'albero a 17 vie con il ramo r15.
  reg [31:0] mul_a, mul_b;
  always @(posedge clk) begin
    mul_a <= rE[{1'b0, dr_rd[2:0]}];
    mul_b <= rE[{1'b0, dr_rm[2:0]}];
  end
  wire [31:0] mul_p = mul_a * mul_b;

  //--------------------------------------------------------------------------
  // PASSO 8 - BARREL SHIFTER A REGISTRO, SU UN CAMMINO TUTTO SUO.
  //
  // Valeva 3,6 ns su 13 livelli del cammino critico (pathdet.tcl, 9 ago 2026)
  // per servire il 2 per mille delle istruzioni. Registrando gli operandi il
  // cammino diventa `sh_val -> shifter -> rE[]`, senza decoder ne' mux di
  // lettura davanti.
  //
  // Espressioni copiate ALLA LETTERA da S_EXEC: il risultato e' bit-identico.
  reg [31:0] sh_val;
  reg [7:0]  sh_amt;
  reg [6:0]  sh_kind;
  reg [31:0] sh_rc;
  reg        sh_cf, sh_cf_we;
  always @* begin
    sh_rc = sh_val; sh_cf = 1'b0; sh_cf_we = 1'b1;
    case (sh_kind)
      OP_LSL2: begin
        if      (sh_amt == 8'd0)  begin sh_rc = sh_val; sh_cf_we = 1'b0; end
        else if (sh_amt <  8'd32) begin sh_rc = sh_val << sh_amt[4:0]; sh_cf = sh_val[32-sh_amt[4:0]]; end
        else if (sh_amt == 8'd32) begin sh_rc = 32'd0; sh_cf = sh_val[0]; end
        else                      begin sh_rc = 32'd0; sh_cf = 1'b0; end
      end
      OP_LSR2: begin
        if      (sh_amt == 8'd0)  begin sh_rc = sh_val; sh_cf_we = 1'b0; end
        else if (sh_amt <  8'd32) begin sh_rc = sh_val >> sh_amt[4:0]; sh_cf = sh_val[sh_amt[4:0]-1]; end
        else if (sh_amt == 8'd32) begin sh_rc = 32'd0; sh_cf = sh_val[31]; end
        else                      begin sh_rc = 32'd0; sh_cf = 1'b0; end
      end
      OP_ASR2: begin
        if      (sh_amt == 8'd0)  begin sh_rc = sh_val; sh_cf_we = 1'b0; end
        else if (sh_amt <  8'd32) begin sh_rc = $signed(sh_val) >>> sh_amt[4:0]; sh_cf = sh_val[sh_amt[4:0]-1]; end
        else                      begin sh_rc = sh_val[31] ? 32'hFFFFFFFF : 32'd0; sh_cf = sh_val[31]; end
      end
      OP_ROR: begin
        if      (sh_amt == 8'd0)     begin sh_rc = sh_val; sh_cf_we = 1'b0; end
        else if (sh_amt[4:0] == 5'd0) begin sh_rc = sh_val; sh_cf = sh_val[31]; end
        else begin sh_rc = (sh_val >> sh_amt[4:0]) | (sh_val << (32-sh_amt[4:0]));
                   sh_cf = sh_val[sh_amt[4:0]-1]; end
      end
      default: sh_cf_we = 1'b0;
    endcase
  end

  reg [19:0] wd_cnt;

  always @(posedge clk) begin
    if (rst) begin
      pc      <= 32'd0;
      state   <= S_FETCH;
      halted  <= 1'b1;
      cb_req  <= 1'b0; cb_id <= 2'd0;
      ir      <= 16'd0;
      dr_op <= OP_LSL1; dr_rd <= 4'd0; dr_rn <= 4'd0; dr_rm <= 4'd0;
      dr_imm11 <= 11'd0;
      brt <= 32'd0;
      fetch_valid <= 64'd0; fetch_lru <= 32'd0;
      bus_req <= 1'b0; bus_we <= 1'b0; bus_be <= 4'd0; bus_sz <= 2'd2;
      bus_addr<= 32'd0; bus_wdata <= 32'd0;
      zn      <= 32'd0; cflag <= 1'b0; vflag <= 1'b0;
      wd_cnt  <= 20'd0;
      r[0]  <= 32'd0; r[1] <= 32'd0; r[2]  <= 32'd0; r[3]  <= 32'd0;
      r[4]  <= 32'd0; r[5] <= 32'd0; r[6]  <= 32'd0; r[7]  <= 32'd0;
      r[8]  <= 32'd0; r[9] <= 32'd0; r[10] <= 32'd0; r[11] <= 32'd0;
      r[12] <= 32'd0; r[13] <= 32'd0; r[14] <= 32'd0; r[15] <= 32'd0;
      rE[0]  <= 32'd0; rE[1] <= 32'd0; rE[2]  <= 32'd0; rE[3]  <= 32'd0;
      rE[4]  <= 32'd0; rE[5] <= 32'd0; rE[6]  <= 32'd0; rE[7]  <= 32'd0;
      rE[8]  <= 32'd0; rE[9] <= 32'd0; rE[10] <= 32'd0; rE[11] <= 32'd0;
      rE[12] <= 32'd0; rE[13] <= 32'd0; rE[14] <= 32'd0; rE[15] <= 32'd0;
    end else if (halted) begin
      // Thumbulator::run() re-initializes every register on each call:
      // SP = cStack, LR = cBase, PC = cStart, r0-r12 = 0.
      if (start) begin
        pc      <= start_pc;
        state   <= S_FETCH;
        halted  <= 1'b0;
        fetch_valid <= 64'd0; fetch_lru <= 32'd0;
        cb_req  <= 1'b0;
        bus_req <= 1'b0; bus_we <= 1'b0;
        zn      <= 32'd0; cflag <= 1'b0; vflag <= 1'b0;
        wd_cnt  <= 20'd0;                  // watchdog azzerato ad ogni run
        r[0]  <= 32'd0; r[1] <= 32'd0; r[2]  <= 32'd0; r[3]  <= 32'd0;
        r[4]  <= 32'd0; r[5] <= 32'd0; r[6]  <= 32'd0; r[7]  <= 32'd0;
        r[8]  <= 32'd0; r[9] <= 32'd0; r[10] <= 32'd0; r[11] <= 32'd0;
        r[12] <= 32'd0; r[13] <= start_sp; r[14] <= start_lr; r[15] <= start_pc;
        rE[0]  <= 32'd0; rE[1] <= 32'd0; rE[2]  <= 32'd0; rE[3]  <= 32'd0;
        rE[4]  <= 32'd0; rE[5] <= 32'd0; rE[6]  <= 32'd0; rE[7]  <= 32'd0;
        rE[8]  <= 32'd0; rE[9] <= 32'd0; rE[10] <= 32'd0; rE[11] <= 32'd0;
        rE[12] <= 32'd0; rE[13] <= start_sp; rE[14] <= start_lr;
        rE[15] <= start_pc;
      end
    end else begin
      // FIX GLOBALE punto 5 - watchdog. Deve stare PRIMA del case: una fine run
      // legittima nello stesso ciclo (OP_BX / OP_BLX2 / S_CB) riassegna halted
      // dopo di qui e vince per last-write-wins.
      wd_cnt <= wd_cnt + 1'b1;
      if (&wd_cnt) halted <= 1'b1;
      // STRADA B: default della porta di scrittura UNICA, valida per tutti gli
      // stati. Prima stavano dentro S_EXEC, che era l'unico a usarla.
      wr_en = 1'b0; wr_idx = dr_rd; wr_val = 32'd0;
      dr_load = 1'b0; dr_sel = 2'd0;   // PASSO 27
      case (state)
        //--------------- FETCH ---------------
        // Thumb: istruzioni a 16 bit, PC allineato a 2 byte. Si legge una word
        // (2 istruzioni) e si seleziona il half in base a pc[1] (come Thumbulator:
        // il PC avanza di 2, le 2 metà della word sono le 2 istruzioni).
        S_FETCH: begin
          if (fc_hit(pc) && !bus_req) begin
            dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
            pc    <= pc + 32'd2;
            state <= S_EXEC;
          end else begin
            bus_addr <= {pc[31:2], 2'b00};
            bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
            if (bus_ack) begin
              dr_load = 1'b1; dr_sel = 2'd3;   // PASSO 27 (era ir_bus)
              // 2-way: riempi la via LRU del set e inverti il bit LRU
              fetch_word [{fetch_lru[pc[6:2]], pc[6:2]}] <= bus_rdata;
              fetch_tag  [{fetch_lru[pc[6:2]], pc[6:2]}] <= pc[31:2];
              fetch_valid[{fetch_lru[pc[6:2]], pc[6:2]}] <= 1'b1;
              fetch_lru[pc[6:2]] <= ~fetch_lru[pc[6:2]];
              // 4 ago 2026 - FETCH A 64 BIT: la ROM sdoppiata (due banchi
              // ruotati, vedi cdf_m10k_memories.sv) restituisce nello STESSO
              // accesso anche la word successiva. Si riempie percio' anche la
              // sua entry: il prossimo fetch sequenziale - che e' il 67-78%
              // dei fetch, misurato - trova il dato in cache e NON entra piu'
              // in S_FETCH.
              //   - i due indirizzi differiscono per il bit 2, quindi cadono
              //     in SET DIVERSI: le due scritture non collidono mai
              //   - `bus_rdata_next_ok` e' 0 col DPC+ e fuori dalla ROM: in
              //     quel caso non si riempie nulla e il comportamento resta
              //     identico a prima
              if (bus_rdata_next_ok) begin
                fetch_word [{fetch_lru[pc_nx[6:2]], pc_nx[6:2]}] <= bus_rdata_next;
                fetch_tag  [{fetch_lru[pc_nx[6:2]], pc_nx[6:2]}] <= pc_nx[31:2];
                fetch_valid[{fetch_lru[pc_nx[6:2]], pc_nx[6:2]}] <= 1'b1;
                fetch_lru[pc_nx[6:2]] <= ~fetch_lru[pc_nx[6:2]];
              end
              bus_req <= 1'b0;
              pc      <= pc + 32'd2;
              state   <= S_EXEC;
            end
          end
        end
        //--------------- EXECUTE ---------------
        S_EXEC: begin
          // BUG CORRETTO (1 ago 2026) - LETTURA DI r15 NELLE FORME A REGISTRO
          // ALTO. In questo core il PC vive in `pc`, non in rE[15]: rE[15] veniva
          // caricato solo all'avvio del run e restava fermo li' per sempre.
          // Un `add rN,pc` / `cmp rN,pc` / `mov rN,pc` (gruppo 0x4400 del
          // decoder, dove dr_rd/dr_rn/dr_rm sono a 4 bit e possono valere 15)
          // leggeva quindi un valore stantio. Thumbulator legge reg_norm[15],
          // che durante l'esecuzione vale instr_addr+4 con bit0 azzerato
          // (read_register): qui e' `pc + 2`, perche' `pc` e' gia' instr_addr+2.
`ifdef ARM_NO_R15FIX
          ra  = rE[dr_rn]; rmv = rE[dr_rm]; rdd = rE[dr_rd];   // solo per bisezione
`else
          ra  = (dr_rn == 4'd15) ? r15_rd : rE[dr_rn];
          rmv = (dr_rm == 4'd15) ? r15_rd : rE[dr_rm];
          rdd = (dr_rd == 4'd15) ? r15_rd : rE[dr_rd];
`endif
          // PASSO 11: b1tgt/b2tgt sono ora wire (vedi il blocco di decode)
          state <= S_FETCH;  // default: prossima istruzione
          // PASSO 8: cattura degli operandi dello scorrimento a registro.
          sh_val <= rdd; sh_amt <= rmv[7:0]; sh_kind <= dr_op;

          // ---- selezione operandi del SOMMATORE CONDIVISO ----
          // Mux stretti a monte di un solo sommatore, al posto di 18 sommatori
          // in parallelo muxati a valle. Vedi la nota sulle dichiarazioni.
          ax = ra; ay = rmv; asub = 1'b0; acin = 1'b0;
          case (dr_op)
            OP_ADD1:                    begin ax = ra;    ay = {29'd0, dr_imm[2:0]}; end
            OP_ADD2:                    begin ax = rdd;   ay = {24'd0, dr_imm[7:0]}; end
            OP_ADD3, OP_ADD4, OP_CMN:   begin ax = ra;    ay = rmv; end
            OP_ADD5:                    begin ax = (pc + 32'd2) & 32'hFFFFFFFC;
                                              ay = {22'd0, dr_imm[7:0], 2'b00}; end
            OP_ADD6:                    begin ax = rE[13]; ay = {22'd0, dr_imm[7:0], 2'b00}; end
            OP_ADD7:                    begin ax = rE[13]; ay = {23'd0, dr_imm[6:0], 2'b00}; end
            OP_ADC:                     begin ax = rdd;   ay = rmv; acin = cflag; end
            OP_SUB1:                    begin ax = ra;    ay = {29'd0, dr_imm[2:0]}; asub = 1'b1; acin = 1'b1; end
            OP_SUB2:                    begin ax = rdd;   ay = {24'd0, dr_imm[7:0]}; asub = 1'b1; acin = 1'b1; end
            OP_SUB3, OP_CMP2, OP_CMP3:  begin ax = ra;    ay = rmv;                 asub = 1'b1; acin = 1'b1; end
            OP_SUB4:                    begin ax = rE[13]; ay = {23'd0, dr_imm[6:0], 2'b00}; asub = 1'b1; acin = 1'b1; end
            OP_CMP1:                    begin ax = ra;    ay = {24'd0, dr_imm[7:0]}; asub = 1'b1; acin = 1'b1; end
            OP_SBC:                     begin ax = rdd;   ay = rmv;                 asub = 1'b1; acin = cflag; end
            OP_NEG:                     begin ax = 32'd0; ay = rmv;                 asub = 1'b1; acin = 1'b1; end
            // 9 ago 2026 - INDIRIZZI DI LOAD/STORE SUL SOMMATORE CONDIVISO.
            // Erano sommatori separati dentro ogni ramo (`rE[dr_rn]+rmv`,
            // `rE[dr_rn]+imm`, ...). Tutte quelle espressioni hanno la forma
            // ax+ay, quindi entrano qui: si risparmiano sommatori e soprattutto
            // l'indirizzo efficace finisce su `asum`, che e' gia' calcolato in
            // S_EXEC. Da li' si puo' presentarlo alla M10K un ciclo prima.
            OP_LDR2, OP_LDRB2, OP_LDRH2, OP_LDRSB, OP_LDRSH,
            OP_STR2, OP_STRB2, OP_STRH2:
                                        begin ax = ra;    ay = rmv; end
            OP_LDR1, OP_STR1:           begin ax = ra;    ay = {18'd0, dr_imm, 2'b00}; end
            OP_LDRB1, OP_STRB1:         begin ax = ra;    ay = {27'd0, dr_imm[4:0]}; end
            OP_LDRH1, OP_STRH1:         begin ax = ra;    ay = {26'd0, dr_imm[4:0], 1'b0}; end
            OP_LDR3:                    begin ax = (pc + 32'd2) & 32'hFFFFFFFC;
                                              ay = {18'd0, dr_imm, 2'b00}; end
            OP_LDR4, OP_STR3:           begin ax = rE[13]; ay = {18'd0, dr_imm, 2'b00}; end
            default: ;
          endcase
          asum = {1'b0, ax} + {1'b0, (asub ? ~ay : ay)} + {32'd0, acin};

          case (dr_op)
            //---- MOV ----
            OP_MOV1: begin begin wr_en = 1'b1; wr_val = {24'd0, dr_imm[7:0]}; end zn <= {24'd0, dr_imm[7:0]}; end // zn only
            OP_MOV2: begin begin wr_en = 1'b1; wr_val = rmv; end zn <= rmv; cflag <= 1'b0; vflag <= 1'b0; end
            // MOV(3)/CPY: nessun flag. Con rd = r15 e' un SALTO
            // (`mov pc,lr` e' un ritorno di funzione Thumb legittimo):
            // Thumbulator fa `rc += 2; rc &= ~1; write_register(15, rc)`, che
            // porta l'istruzione successiva a rm & ~1 - qui `pc <= rmv & ~1`.
            OP_MOV3: begin
              if (dr_rd == 4'd15) pc <= rmv & 32'hFFFFFFFE;
              else               begin wr_en = 1'b1; wr_val = rmv; end
            end
            OP_CPY:  begin
              if (dr_rd == 4'd15) pc <= rmv & 32'hFFFFFFFE;
              else               begin wr_en = 1'b1; wr_val = rmv; end
            end

            //---- ADD (zn + cflag/vflag fedeli a do_cvflag) ----
            // ADD Rd,Rn,#imm3: source is Rn (ra), not Rd. (ADD2 below is the
            // two-operand ADD Rd,#imm8 form which does read Rd.)
            OP_ADD1: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= ~(ra[31] ^ 1'b0) & (ra[31] ^ rc[31]); end
            OP_ADD2: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= ~(rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_ADD3: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= ~(ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            // ADD(4) registri alti: nessun flag. Con rd = r15 e' un salto
            // calcolato (idioma delle tabelle di switch in Thumb-1).
            OP_ADD4: begin rc = asum[31:0];
              if (dr_rd == 4'd15) pc <= rc & 32'hFFFFFFFE;
              else               begin wr_en = 1'b1; wr_val = rc; end
            end
            OP_ADD5: begin rc = asum[31:0]; begin wr_en = 1'b1; wr_val = rc; end end
            OP_ADD6: begin rc = asum[31:0]; begin wr_en = 1'b1; wr_val = rc; end end
            OP_ADD7: begin begin wr_en = 1'b1; wr_idx = 4'd13; wr_val = asum[31:0]; end end

            //---- SUB ----
            // SUB Rd,Rn,#imm3: source is Rn (ra), not Rd.
            OP_SUB1: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= (ra[31] ^ 1'b0) & (ra[31] ^ rc[31]); end
            OP_SUB2: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= (rdd[31] ^ 1'b0) & (rdd[31] ^ rc[31]); end
            OP_SUB3: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_SUB4: begin begin wr_en = 1'b1; wr_idx = 4'd13; wr_val = asum[31:0]; end end

            //---- ADC / SBC ----
            OP_ADC: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= ~(rdd[31] ^ rmv[31]) & (rdd[31] ^ rc[31]); end
            OP_SBC: begin rc = asum[31:0];
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= (rdd[31] ^ rmv[31]) & (rdd[31] ^ rc[31]); end

            //---- LOGIC (zn only, C/V invariati salvo dove indicato) ----
            OP_AND: begin rc = rdd & rmv; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_ORR: begin rc = rdd | rmv; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_EOR: begin rc = rdd ^ rmv; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_BIC: begin rc = rdd & ~rmv; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_MVN: begin rc = ~rmv; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_CMN: begin rc = asum[31:0]; zn <= rc; cflag <= asum[32];
              vflag <= ~(ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_CMP1: begin rc = asum[31:0]; zn <= rc; cflag <= asum[32];
              vflag <= (ra[31] ^ 1'b0) & (ra[31] ^ rc[31]); end
            OP_CMP2: begin rc = asum[31:0]; zn <= rc; cflag <= asum[32];
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_CMP3: begin rc = asum[31:0]; zn <= rc; cflag <= asum[32];
              vflag <= (ra[31] ^ rmv[31]) & (ra[31] ^ rc[31]); end
            OP_TST: begin rc = rdd & rmv; zn <= rc; end
            OP_NEG: begin rc = asum[31:0]; begin wr_en = 1'b1; wr_val = rc; end zn <= rc; cflag <= asum[32];
              vflag <= (1'b0 ^ rmv[31]) & (1'b0 ^ rc[31]); end
            // TIMING (1 ago 2026) - il moltiplicatore NON passa piu' dalle mux
            // condivise rdd/rmv. Cammino critico MISURATO su clk_vid:
            //   ir[13] -> dec|Equal11~0 -> rmv~109 -> rmv~113 -> Mult0~405
            //          -> catena di carry -> rE[2][31]      slack -0.616 ns
            // di cui 4.78 ns nell'albero di mux dell'operando e 11.7 ns nel
            // moltiplicatore, su un budget di 17.46 ns.
            // MUL sta nel gruppo 0x4000 del decoder, dove dr_rd e dr_rm valgono
            // {1'b0, op[2:0]} e {1'b0, op[5:3]}: sono SEMPRE <= 7, quindi r15
            // non puo' esserne un operando e la mux a 17 vie (con il ramo r15)
            // e' inutile qui. Indicizzando con i soli 3 bit bassi il
            // moltiplicatore riceve una mux 8:1 dedicata invece dell'albero
            // completo. Nessun cambiamento funzionale.
            // PASSO 4: il prodotto arriva nel ciclo dopo, da `mul_p`.
            // Gli operandi sono gia' stati catturati in mul_a/mul_b su questo
            // stesso fronte, e `ir` (quindi dr_rd) resta valido in S_MUL.
            OP_MUL: state <= S_MUL;

            //---- SHIFT (zn + cflag=carry-out; vflag invariato) ----
            OP_LSL1: begin if (dr_imm[4:0]==5'd0) rc = rmv;
              else begin rc = rmv << dr_imm[4:0]; cflag <= rmv[32-dr_imm[4:0]]; end
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            // BUG CORRETTO (1 ago 2026) - SCORRIMENTI A REGISTRO >= 32.
            // Il conteggio si prendeva con rmv[4:0], quindi 32 diventava 0
            // ("non spostare") e 33 diventava 1. Thumbulator maschera a 8 bit e
            // distingue tre casi (Thumbulator.cxx lsl2:2120 lsr2:2175
            // asr2:1318 ror:2498): < 32 normale, == 32 e > 32 azzerano il
            // registro (o lo saturano al segno per ASR) con carry definito.
            OP_LSL2: state <= S_SHIFT;
            OP_LSR1: begin if (dr_imm[4:0]==5'd0) begin rc = 32'd0; cflag <= rmv[31]; end
              else begin rc = rmv >> dr_imm[4:0]; cflag <= rmv[dr_imm[4:0]-1]; end
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_LSR2: state <= S_SHIFT;
            OP_ASR1: begin if (dr_imm[4:0]==5'd0) begin rc = rmv[31] ? 32'hFFFFFFFF : 32'd0; cflag <= rmv[31]; end
              else begin rc = $signed(rmv) >>> dr_imm[4:0]; cflag <= rmv[dr_imm[4:0]-1]; end
              begin wr_en = 1'b1; wr_val = rc; end zn <= rc; end
            OP_ASR2: state <= S_SHIFT;
            // ROR: Thumbulator maschera prima a 8 bit (0 = nessun effetto, C
            // invariato) e POI a 5 bit; se il residuo e' 0 il valore non ruota
            // ma il carry prende comunque il bit 31.
            OP_ROR: state <= S_SHIFT;
            OP_REV:  begin begin wr_en = 1'b1; wr_val = {rmv[7:0],rmv[15:8],rmv[23:16],rmv[31:24]}; end end
            OP_REV16:begin begin wr_en = 1'b1; wr_val = {rmv[23:16],rmv[31:24],rmv[7:0],rmv[15:8]}; end end
            OP_REVSH:begin begin wr_en = 1'b1; wr_val = {{16{rmv[7]}},rmv[7:0],rmv[15:8]}; end end
            OP_SXTB: begin begin wr_en = 1'b1; wr_val = {{24{rmv[7]}},rmv[7:0]}; end end
            OP_SXTH: begin begin wr_en = 1'b1; wr_val = {{16{rmv[15]}},rmv[15:0]}; end end
            OP_UXTB: begin begin wr_en = 1'b1; wr_val = {24'd0, rmv[7:0]}; end end
            OP_UXTH: begin begin wr_en = 1'b1; wr_val = {16'd0, rmv[15:0]}; end end

            //---- BRANCH ----
            OP_BEQ,OP_BNE,OP_BCS,OP_BCC,OP_BMI,OP_BPL,OP_BVS,OP_BVC,
            OP_BHI,OP_BLS,OP_BGE,OP_BLT,OP_BGT,OP_BLE: begin
              if (branch_taken) pc <= brt;   // PASSO 33
            end
            OP_B2:    begin pc <= brt; end   // PASSO 33
            // BX/BLX to an EVEN address means 32-bit ARM code, which this
            // core (like Thumbulator) does not execute: it is either one of
            // the driver's music callbacks (trap and return via LR) or
            // the end of the run (e.g. main() returning to LR = cBase).
            OP_BX:    begin
              if (rmv[0]) pc <= rmv & ~32'd1;
              else if ((pc + 32'd2) == cb_addr0) begin cb_id <= 2'd0; cb_req <= 1'b1; state <= S_CB; end
              else if ((pc + 32'd2) == cb_addr1) begin cb_id <= 2'd1; cb_req <= 1'b1; state <= S_CB; end
              else if ((pc + 32'd2) == cb_addr2) begin cb_id <= 2'd2; cb_req <= 1'b1; state <= S_CB; end
              else if ((pc + 32'd2) == cb_addr3) begin cb_id <= 2'd3; cb_req <= 1'b1; state <= S_CB; end
              else halted <= 1'b1;
            end
            OP_BLX2:  begin
              if (rmv[0]) begin begin wr_en = 1'b1; wr_idx = 4'd14; wr_val = pc | 32'd1; end pc <= rmv & ~32'd1; end
              else halted <= 1'b1; // Thumbulator: blx to ARM ends the run
            end
            // BL a 32 bit: prima half memorizza parziale in LR (Thumbulator Op::bl)
            OP_BL:    begin begin wr_en = 1'b1; wr_idx = 4'd14; wr_val = ({{21{ir[10]}}, ir[10:0]} << 12) + pc; end end
            // seconda half completa target + LR (Thumbulator Op::blx_thumb/arm)
            OP_BLX_THUMB: begin begin wr_en = 1'b1; wr_idx = 4'd14; wr_val = pc | 32'd1; end
              pc <= rE[14] + ({21'd0, ir[10:0]} << 1) + 32'd2; end
            // BUG CORRETTO (1 ago 2026). Qui c'era
            //     pc_full = {rE[14] + (imm11<<1) + 2, 2'b00};
            // cioe' una CONCATENAZIONE, che vale una MOLTIPLICAZIONE PER 4.
            // Thumbulator (blx_arm:1515) fa invece
            //     rb = LR + (imm11<<1); rb &= 0xFFFFFFFC; rb += 2;
            // e write_register(15, rb) porta l'istruzione successiva a
            // (LR_thumbulator + imm11*2) & ~3. Con LR di questo core (2 in meno
            // di quello di Thumbulator, vedi OP_BL) il target corretto e'
            // ((r14 + imm11*2 + 2) & ~3): un MASCHERAMENTO, non un prodotto.
            OP_BLX_ARM:   begin begin wr_en = 1'b1; wr_idx = 4'd14; wr_val = pc | 32'd1; end
              pc <= (rE[14] + ({21'd0, ir[10:0]} << 1) + 32'd2) & 32'hFFFFFFFC;
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
              blk_addr  <= rE[13] - {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              blk_we    <= 1'b1; blk_base <= 4'd13; blk_wb <= 1'b1;
              blk_wbval <= rE[13] - {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              state <= S_BLK;
            end
            OP_POP: begin
              blk_rem   <= {ir[8], ir[7:0]};
              blk_addr  <= rE[13];
              blk_we    <= 1'b0; blk_base <= 4'd13; blk_wb <= 1'b1;
              blk_wbval <= rE[13] + {25'd0, popcount9({ir[8], ir[7:0]}), 2'b00};
              state <= S_BLK;
            end

            //---- LOAD/STORE singoli (bus bridge, indirizzo allineato a word) ----
            OP_LDR1: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR2: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR3: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=32'd0; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDR4: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;   bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=32'd0; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRB1:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;bus_sz<=2'd0; bus_be<=be_of(asum[31:0]); bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRB2:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(asum[31:0]);       bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRH1:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC; bus_sz<=2'd1; bus_be<=beh_of(asum[31:0]); bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRH2:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(asum[31:0]);       bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRSB:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(asum[31:0]);       bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_LDRSH:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(asum[31:0]);       bus_wdata<=rE[dr_rd]; bus_we<=1'b0; bus_req<=1'b1; state<=S_MEM; end
            OP_STR1: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC; bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=rE[dr_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STR2: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=rE[dr_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STR3: begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;   bus_sz<=2'd2; bus_be<=4'b1111; bus_wdata<=rE[dr_rd]; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            // Sub-word stores must place data in the addressed byte lane, not
            // in the low lane: byte-enables select the lane, so replicate the
            // byte/half across all lanes so the enabled lane carries the value.
            OP_STRB1:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;bus_sz<=2'd0; bus_be<=be_of(asum[31:0]); bus_wdata<={4{rE[dr_rd][7:0]}};  bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRB2:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd0; bus_be<=be_of(asum[31:0]);       bus_wdata<={4{rE[dr_rd][7:0]}};  bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRH1:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC; bus_sz<=2'd1; bus_be<=beh_of(asum[31:0]); bus_wdata<={2{rE[dr_rd][15:0]}}; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_STRH2:begin mem_addr<=asum[31:0]; bus_addr<=asum[31:0] & 32'hFFFFFFFC;       bus_sz<=2'd1; bus_be<=beh_of(asum[31:0]);       bus_wdata<={2{rE[dr_rd][15:0]}}; bus_we<=1'b1; bus_req<=1'b1; state<=S_MEM; end
            OP_LDMIA: begin
              blk_rem   <= {1'b0, ir[7:0]};
              blk_addr  <= rE[dr_rn];
              blk_we    <= 1'b0; blk_base <= dr_rn;
              blk_wb    <= ((ir[7:0] & (8'h01 << dr_rn[2:0])) == 8'h00);
              blk_wbval <= rE[dr_rn] + {25'd0, popcount9({1'b0, ir[7:0]}), 2'b00};
              state <= S_BLK;
            end
            // BUG CORRETTO (1 ago 2026): STMIA scrive SEMPRE il registro base
            // (Thumbulator.cxx:2586 `write_register(rn, sp)` incondizionato).
            // L'eccezione "niente writeback se la base e' nella lista" vale
            // per la sola LDMIA (:1930) ed era stata applicata anche qui: con
            // `stmia rN!,{rN,...}` la base restava ferma e il ciclo di copia
            // successivo ripartiva dallo stesso indirizzo.
            OP_STMIA: begin
              blk_rem   <= {1'b0, ir[7:0]};
              blk_addr  <= rE[dr_rn];
              blk_we    <= 1'b1; blk_base <= dr_rn;
              blk_wb    <= 1'b1;
              blk_wbval <= rE[dr_rn] + {25'd0, popcount9({1'b0, ir[7:0]}), 2'b00};
              state <= S_BLK;
            end

            default: begin end // dr_op==0 (invalid): no-op, resta in FETCH
          endcase
          // FAST PATH (Step 1 speed): l'istruzione appena eseguita non tocca
          // il bus ne' il PC (fast_ok) e la prossima e' gia' in cache: fondi
          // il fetch qui e resta in S_EXEC (1 clk/istruzione). Nonblocking
          // last-write-wins: nessun ramo whitelisted da fast_ok assegna
          // pc/state/ir, quindi queste assegnazioni non confliggono. bus_req
          // e' sempre 0 in S_EXEC (ogni stato che lo alza lo azzera sull'ack).
          if (fast_ok && fc_hit(pc)) begin
            dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
            pc    <= pc + 32'd2;
            state <= S_EXEC;
          end else if (fast_ok) begin
            // Step 3 speed: miss in fallthrough — emetti la richiesta di
            // fetch GIA' in questo ciclo invece di bruciare il primo ciclo
            // di S_FETCH a registrare bus_req. Il ramo miss di S_FETCH
            // ri-registra gli stessi valori in modo idempotente. fast_ok
            // garantisce che pc non venga ridiretto in questo ciclo e che
            // l'istruzione non stia gia' usando il bus.
            bus_addr <= {pc[31:2], 2'b00};
            bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
          end else if (br_fast_take) begin
            // FAST PATH branch (Step 7 speed): salto preso verso codice Thumb
            // con target noto in questo ciclo (b1tgt/b2tgt/rmv). Se la word
            // del target e' in cache, carica ir subito e resta in S_EXEC
            // (elimina il ciclo S_FETCH post-branch: 2 branch presi per
            // iterazione nel loop fill = ~20% dei clk). Su miss, emissione
            // anticipata della richiesta come Step 3; pc resta quello
            // assegnato dal ramo del case (last-write-wins solo sul hit).
            // Esclusi (restano legacy): BL/BLX_THUMB/BLX_ARM, POP {..,pc},
            // S_CB, e da oggi anche BX/BLX2 (vedi PASSO 5).
            // PASSO 5: mux 2:1 comandato da un confronto diretto su ir,
            // non dal decoder. BX/BLX2 sono usciti dal fast path: il loro
            // bersaglio arriva dal file registri e ci trascinava dentro tutto
            // l'albero di mux dei registri.
            // PASSO 11: br_tgt e' ora un wire
            // PASSO 21: anche il controllo di presenza parte dai registri
            if (fc_hit(brt)) begin   // PASSO 33: un solo confronto
              dr_load = 1'b1; dr_sel = 2'd1;   // PASSO 27 (era ir_brt)
              pc    <= brt + 32'd2;
              state <= S_EXEC;
            end else begin
              bus_addr <= {brt[31:2], 2'b00};
              bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
            end
          end
        end
        //--------------- CB (music-callback trap handshake) ---------------
        S_CB: begin
          if (cb_ack) begin
            cb_req <= 1'b0;
            // _GetWavePtr restituisce in r2
            if (cb_id == 2'd2) begin wr_en = 1'b1; wr_idx = 4'd2; wr_val = cb_ret; end
            // Step 2 speed: se la word dell'indirizzo di ritorno e' in cache,
            // salta S_FETCH (il return addr va calcolato come temp blocking
            // perche' pc viene riassegnato in questo stesso ciclo).
            // PASSO 11: lr_pc e' ora un wire
            // PASSO 32: il ramo veloce e' STATO TOLTO. Il ritorno dalla
            // callback passa sempre per S_FETCH: costa un ciclo, e su
            // Draconian sono 3 cicli su 169.280 (+0,0018%). In cambio sparisce
            // il lookup `fc_ir(lr_pc)`, che occupava il cammino peggiore.
            pc     <= lr_pc;
            state  <= S_FETCH;
          end
        end
        //--------------- MEM (attesa ack, transfer singolo) ---------------
        S_MEM: begin
          if (bus_we && fc_hit(bus_addr))
            begin
              // 4 ago 2026: la via era calcolata chiamando fc_way() DENTRO
              // l'indice del lato sinistro di un assegnamento non bloccante.
              // Il compilatore C++ ci va in errore interno (V3Delayed:
              // "Multiple Write refs on LHS of NBA") perche' la funzione scrive la propria
              // variabile interna. Calcolata prima in un temporaneo bloccante:
              // stesso valore, stesso ciclo, nessun cambio di comportamento.
              inv_way = fc_way(bus_addr);
              fetch_valid[{inv_way, bus_addr[6:2]}] <= 1'b0;
            end
          if (bus_ack) begin
            bus_req <= 1'b0;
            if (!bus_we) begin
              if      (bus_sz==2'd2) ld_val_comb = bus_rdata;
              else if (bus_sz==2'd1) ld_val_comb = {16'd0, bus_rdata[(mem_addr[1]*16) +: 16]};
              else                   ld_val_comb = {24'd0, bus_rdata[(mem_addr[1:0]*8) +: 8]};
              if (dr_op==OP_LDRSB) ld_val_comb = {{24{ld_val_comb[7]}},  ld_val_comb[7:0]};
              if (dr_op==OP_LDRSH) ld_val_comb = {{16{ld_val_comb[15]}}, ld_val_comb[15:0]};
              begin wr_en = 1'b1; wr_val = ld_val_comb; end
            end
            // Step 2 speed: pc e' invariato durante S_MEM; se la prossima
            // istruzione e' in cache salta S_FETCH. Guardia self-modifying:
            // se QUESTO store sta invalidando proprio la word in cache (riga
            // sopra, nonblocking) non usare il contenuto stantio.
            if (fc_hit(pc) &&
                !(bus_we && bus_addr[31:2] == pc[31:2])) begin
              dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
              pc    <= pc + 32'd2;
              state <= S_EXEC;
            end else begin
              // Step 3 speed: miss — invece di rilasciare il bus e perdere
              // un ciclo, ritargetizza la richiesta sull'indirizzo di fetch
              // (CRITICO: azzerare we e ripristinare be/sz dopo uno store).
              // Il bridge ignora req nel ciclo di ack (!bus_ack in guardia)
              // e riparte il ciclo dopo: handshake accorciato di 1 ciclo.
              bus_addr <= {pc[31:2], 2'b00};
              bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
              state <= S_FETCH;
            end
          end
        end
        //--------------- BLK (transfer multi-registro PUSH/POP/LDMIA/STMIA) ---------------
        S_BLK: begin
          if (bus_we && fc_hit(bus_addr))
            begin
              // 4 ago 2026: la via era calcolata chiamando fc_way() DENTRO
              // l'indice del lato sinistro di un assegnamento non bloccante.
              // Il compilatore C++ ci va in errore interno (V3Delayed:
              // "Multiple Write refs on LHS of NBA") perche' la funzione scrive la propria
              // variabile interna. Calcolata prima in un temporaneo bloccante:
              // stesso valore, stesso ciclo, nessun cambio di comportamento.
              inv_way = fc_way(bus_addr);
              fetch_valid[{inv_way, bus_addr[6:2]}] <= 1'b0;
            end
          if (blk_rem != 9'd0) begin
            idx   = lsb_index(blk_rem);
            regno = (idx==4'd8) ? ((dr_op==OP_PUSH) ? 4'd14 : 4'd15) : idx[3:0];
            bus_addr <= blk_addr;
            bus_sz   <= 2'd2; bus_be <= 4'b1111;
            bus_we   <= blk_we;
            bus_wdata<= rE[regno];
            bus_req  <= 1'b1;
            if (bus_ack) begin
              bus_req <= 1'b0;
              if (!blk_we) begin
                if (regno==4'd15) pc <= bus_rdata & ~32'd1;
                else              begin wr_en = 1'b1; wr_idx = regno; wr_val = bus_rdata; end
              end
              blk_rem  <= blk_rem & ~(9'd1 << idx[3:0]);
              blk_addr <= blk_addr + 32'd4;
              if ((blk_rem & ~(9'd1 << idx[3:0])) == 9'd0) begin
                if (blk_wb) begin
                  r[blk_base] <= blk_wbval; rE[blk_base] <= blk_wbval;
                end
                // Step 2 speed: uscita rapida verso S_EXEC se la prossima
                // istruzione e' in cache. MAI quando l'ultimo beat carica il
                // PC (POP {..,pc}: pc riassegnato in questo ciclo) e mai se
                // questo store sta invalidando la word in cache (stantia).
                if (regno != 4'd15 && fc_hit(pc) &&
                    !(blk_we && bus_addr[31:2] == pc[31:2])) begin
                  dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
                  pc    <= pc + 32'd2;
                  state <= S_EXEC;
                end else if (regno != 4'd15) begin
                  // Step 3 speed: ritarget della richiesta sul fetch (mai
                  // quando l'ultimo beat carica il PC: indirizzo non pronto).
                  bus_addr <= {pc[31:2], 2'b00};
                  bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
                  state <= S_FETCH;
                end else state <= S_FETCH;
              end
            end
          end else begin
            state <= S_FETCH;
          end
        end
        // PASSO 8 - secondo ciclo dello scorrimento a registro.
        // Stesso schema di S_MUL, fast path compreso: senza, costerebbe DUE
        // cicli in piu' invece di uno.
        S_SHIFT: begin
          begin wr_en = 1'b1; wr_val = sh_rc; end
          zn      <= sh_rc;
          if (sh_cf_we) cflag <= sh_cf;
          state   <= S_FETCH;
          if (fc_hit(pc)) begin
            dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
            pc    <= pc + 32'd2;
            state <= S_EXEC;
          end else begin
            bus_addr <= {pc[31:2], 2'b00};
            bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
          end
        end
        // PASSO 4 - secondo ciclo della MUL: il prodotto e' pronto su mul_p.
        //
        // Qui si rifa' il FAST PATH che S_EXEC non ha potuto applicare: senza,
        // la MUL costerebbe DUE cicli in piu' invece di uno (uno per S_MUL e
        // uno per il fetch non fuso). Su Spiders le MUL sono il 24 per mille
        // (misura [opmix]), quindi il secondo ciclo si sarebbe visto.
        S_MUL: begin
          begin wr_en = 1'b1; wr_val = mul_p; end
          zn      <= mul_p;
          state   <= S_FETCH;
          if (fc_hit(pc)) begin
            dr_load = 1'b1; dr_sel = 2'd0;   // PASSO 27 (era ir_pc)
            pc    <= pc + 32'd2;
            state <= S_EXEC;
          end else begin
            // emissione anticipata della richiesta di fetch, come il ramo
            // `else if (fast_ok)` di S_EXEC: il ramo miss di S_FETCH
            // ri-registra gli stessi valori in modo idempotente.
            bus_addr <= {pc[31:2], 2'b00};
            bus_sz   <= 2'd2; bus_be <= 4'b1111; bus_we <= 1'b0; bus_req <= 1'b1;
          end
        end
        default: state <= S_FETCH;
      endcase
      // STRADA B: UNICO punto in cui il core scrive nel banco registri.
      // Sta fuori dal `case`, quindi vale per ogni stato. L'unica eccezione e'
      // il writeback della base in S_BLK, che nello stesso ciclo scrive un
      // secondo registro (vedi l'intestazione di passo14_wrunica.py).
      // PASSO 27: UNICO punto in cui si caricano `ir` e i registri di
      // decodifica. Sta fuori dal `case`, come la porta di scrittura del
      // banco: i rami dichiarano solo QUALE sorgente, con due bit.
      if (dr_load) begin
        ir       <= ir_src[dr_sel];
        dr_op    <= pd_op[dr_sel];    dr_rd    <= pd_rd[dr_sel];
        dr_rn    <= pd_rn[dr_sel];    dr_rm    <= pd_rm[dr_sel];
        dr_imm11 <= pd_imm11[dr_sel];
        brt      <= pre_brt[dr_sel];   // PASSO 33
      end
      if (wr_en) begin
        r[wr_idx] <= wr_val; rE[wr_idx] <= wr_val;   // PASSO 26b
      end
    end
  end
`ifdef VERILATOR
  // PASSO 26b - i due banchi DEVONO coincidere sempre.
  integer chk26;
  always @(posedge clk) if (!rst)
    for (chk26 = 0; chk26 < 16; chk26 = chk26 + 1)
      if (rE[chk26] !== r[chk26])
        $error("[26b] rE[%0d]=%h != r[%0d]=%h", chk26, rE[chk26],
               chk26, r[chk26]);
`endif
endmodule
/* verilator lint_on BLKSEQ */
/* verilator lint_on WIDTHEXPAND */
/* verilator lint_on WIDTHTRUNC */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
`default_nettype wire
