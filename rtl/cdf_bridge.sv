// CDF/CDFJ/CDFJ+ cartridge bridge: 6502-side protocol + ARM bus slave.
// Semantics follow Stella's CartCDF.cxx (the reference the games are written
// against). See cdf_family_params.sv for the per-family constants.
//
// 6502 reads $1000-$1FFF serve the program ROM byte at
//   prog_off + bank*4KB + offset            (prog_off: 4KB, CDFJ+: 2KB)
// with these read side effects (Stella peek(), same order):
//   1) pending FASTJMP operand -> byte from jump stream, ptr += 1.0
//   2) byte 0x4C in fast-fetch mode: probe next two program bytes; if
//      (b1 & jump_mask)==0 && b2==0 arm a 2-byte fast jump from stream 33+b1
//   3) pending LDA# (LDX#/LDY#) operand <= amp_stream: serve datastream
//      (ptr at ds_base+idx*4 -> display byte at RAM 2KB + ptr>>20 [+: >>16],
//      ptr += increment<<12 [CDFJ+: <<8], written back); == amp_stream:
//      serve the AMPLITUDE value (background-computed 3-voice waveform sum
//      or digital sample)
//   4) clear pending operand address
//   5) hotspots $FF4-$FFB switch banks (reads AND writes, like Stella)
//   6) byte 0xA9 (LDA #) arms fast fetch for the next address
// 6502 writes: $FF0 DSWRITE / $FF1 DSPTR (COMMSTREAM pointer lives in Harmony
// RAM, RMW here), $FF2 SETMODE, $FF3 CALLFN (254/255 -> one ARM run), banks.
//
// The ARM (thumb_core) masters port A: region 0 ROM (read-only), region 4
// Harmony RAM, region 0xE minimal LPC timer/SysTick/MAMCR stub (so driver
// polling loops terminate), 0xD/0xF ignored. Music callbacks (_SetNote /
// _ResetWave / _GetWavePtr / _SetWaveSize) are serviced via cb_* from
// thumb_core's BX-to-ARM trap.
//
// Timing: le porte M10K hanno il registro di INDIRIZZO ma NON quello di uscita
// (1 ciclo di latenza in lettura, su entrambe le porte dal 7 agosto 2026).
// Un ciclo CPU dura 12 clk (clk_sys 14.318 MHz / phi 1.19 MHz). L'engine parte
// una volta per ciclo CPU al cambio dell'indirizzo di cartuccia (fronte e1); il
// percorso di serve piu' lungo (datastream) produce m6502_dout a e7 e la CPU
// campiona a e12. La regola di temporizzazione completa - e l'elenco di TUTTI i
// consumatori della porta B, che vanno adeguati insieme - sta sopra la FSM.
`default_nettype none
module cdf_bridge #(
  parameter ROM_FILE="", RAM_FILE="", parameter ROM_DEPTH=131072,
  // RAM_DEPTH: 2 ago 2026 da 8192 a 32768. Stella dichiara 32 KB di RAM Harmony
  // (CartCDF.hxx:300) e il pannello CartridgeCDFJ+ del debugger lo scrive:
  // "32K RAM", "$0800-$7FFF - 30K Data Stream storage". Con 8192 la storage era
  // 6144 byte e ogni puntatore oltre quel limite veniva AZZERATO (vedi
  // wf_disp_byte/disp_ok): schermata di scoring rovinata in Spiders Arcade,
  // audio rovinato in Tutankham Arcade.
  parameter RAM_DEPTH=32768, parameter DISP_DEPTH=4096, // DISP_DEPTH legacy, unused
  parameter bit SIM_TEST_HOOKS=0,
  // FASE9 §3.3 - rete di sicurezza dell'attesa selettiva: numero massimo di cicli
  // clk_sys per cui un serve datastream puo' restare in attesa dell'ARM. ~1 frame
  // NTSC (262143 clk_sys); l'attesa legittima peggiore misurata e' ~112k, quindi
  // non scatta mai nel funzionamento normale. Serve SOLO a rompere un deadlock:
  // se l'ARM si blocca, il 6507 fermo non puo' avanzare fino a scrivere VBLANK=1,
  // quindi ~vblank_sw non si auto-rilascerebbe mai. Deve restare FINITO.
  parameter [17:0] WAIT_MAX = 18'd262143,
  // Margine utile per l'attesa DENTRO un ciclo del 6507 (12 clk_sys).
  // Il valore va scelto perche' la sonda [abort] dia ZERO troncature: e'
  // l'unica verifica che conta. MISURATO il 7 agosto 2026:
  //   4 -> 0 troncature (Lode Runner e Draconian), traccia d'oro invariata
  //        salvo ds_serves 2080 -> 2081, cioe' esattamente il serve che prima
  //        veniva buttato
  //   5 -> il serve sfora il ciclo e le troncature TORNANO (lr 5, dr 1)
  //   3 -> traccia d'oro stravolta (ds_serves 3915, tia_writes 8487): e' la
  //        firma della regressione del 6 agosto, da NON usare
  // Quindi 4 non e' un numero magico: e' il MASSIMO di attesa per cui il serve
  // fa ancora in tempo. La catena e': S_R1, S_DEC, 5 x S_DWAIT, S_DS0/2/3/6 ->
  // ultimo stato al ciclo 11, il 6507 campiona a 12.
  //
  // VALVOLA DI SICUREZZA: portando questo parametro a 18'd262143 (= WAIT_MAX)
  // la condizione non scatta mai e il comportamento torna ESATTAMENTE quello
  // di prima del 7 agosto, senza toccare la macchina a stati. Serve se il
  // collaudo su hardware dovesse dare problemi: si isola l'attesa a budget
  // lasciando in piedi la porta B a 1 ciclo, che e' verificata neutra a parte.
  parameter [17:0] DWAIT_BUDGET = 18'd4,
  // PASSO ARSP (attesa sulla risposta del COMMSTREAM). ARSP_BUDGET e' solo
  // una rete di sicurezza: l'uscita normale e' la FINE della corsa ARM.
  // Con ARSP_EN=0 il comportamento torna esattamente quello di prima.
  parameter bit    ARSP_EN     = 1'b1,
  // 0 = attesa su TUTTO il ramo CDF (scelta del PM, 13 ago 2026, cosi' ne
  //     beneficiano anche Lode Runner e Mappy).
  // 1 = solo CDFJ+ (il ramo di Spiders), variante piu' conservativa.
  // MISURATO con 0: 37 titoli su 38 con video IDENTICO; `zoo` cambia 60-109
  // pixel su 72.000, ed e' la SOLA cifra del punteggio che cicla di colore
  // presa in un punto diverso del ciclo (stessa sequenza bianco x4 / ciano
  // x2 / azzurro x2, sfasata di 4 fotogrammi; geometria identica, scritture
  // TIA identiche). Il PM ha verificato su Gopher che li' il punteggio
  // cambia colore davvero: `zoo` e' a posto.
  parameter bit    ARSP_PLUS_ONLY = 1'b0,
  parameter [17:0] ARSP_BUDGET = 18'd8192
)(
  input wire clk,               // clk_sys: 6502 engine / loader / control
  input wire clk_vid,           // Livello 1: dominio ARM (thumb_core) - porte A M10K + adapter bus ARM
  input wire rst,               // reset nel dominio clk_sys (engine/loader)
  input wire rst_vid,           // reset gia' sincronizzato in clk_vid (adapter ARM)
  // 6502 bus (addr/din/rwn stable for the whole CPU cycle)
  input wire [15:0] m6502_addr, input wire [7:0] m6502_din, input wire m6502_rwn,
  output reg [7:0] m6502_dout,
  // ARM bus slave (ora servito localmente su clk_vid dall'adapter, NON piu' via
  // handshake CDC: bus_req/bus_ack/bus_rdata sono nel dominio clk_vid)
  input wire [31:0] bus_addr, input wire [31:0] bus_wdata, output wire [31:0] bus_rdata,
  // STADIO 2: indirizzo efficace ANTICIPATO di un ciclo (vedi thumb_core)
  input wire [31:0] bus_addr_pre, input wire bus_req_pre,
  input wire [31:0] bus_addr_pre_nx,   // PASSO 15: indirizzo + 4
  input wire [3:0] bus_be, input wire bus_we, input wire [1:0] bus_sz, input wire bus_req,
  output wire bus_ack,
  // word successiva a bus_addr, gratis dalla ROM sdoppiata (vedi cdf_m10k_memories)
  output wire [31:0] bus_rdata_next,
  output wire        bus_rdata_next_ok,
  // ARM run control / music callbacks
  output reg        callfn,      // 1-clk pulse on $FF3 write
  output reg [7:0]  callfn_val,
  input wire        cb_req,      // level from thumb_core until cb_ack
  input wire [1:0]  cb_id,       // 0 SetNote, 1 ResetWave, 2 GetWavePtr, 3 SetWaveSize
  input wire [31:0] cb_v1, input wire [31:0] cb_v2,
  output reg [31:0] cb_ret,
  output reg        cb_ack,
  // configuration
  input wire [1:0] family_sel,
  input wire ldx_en, input wire ldy_en,   // CDFJ+ LDX/LDY fast fetch enables
  input wire [15:0] ff_offset,            // CDFJ+ fast-fetch offset cell (0=none)
  // Livello 3 (ping-pong Display Data): selettore del buffer "front" che il 6502
  // legge. L'ARM scrive sempre il buffer "back" (~disp_bank). Generato dalla
  // logica A/B/C nel subsystem (clk_sys). Solo la regione Display Data
  // (word 512..1535) e' duplicata; driver/pointer/C-vars restano condivisi.
  input wire disp_bank,

  // FASE9 - attesa selettiva. ds_wait_arm (livello, da atari2600_arm_subsystem):
  // "siamo nel Kernel e un run ARM e' ancora attivo". Il bridge lo combina con la
  // terza condizione ("sto per servire un datastream") e alza ds_wait, che in
  // top.sv toglie RDY al 6507 per il tempo strettamente necessario.
  input wire ds_wait_arm,
  // PASSO ARSP: "una corsa ARM e' in volo E la TIA NON sta disegnando".
  // ds_wait_arm e' il complementare (Kernel) e li' l'attesa NON si puo'
  // fare: un ciclo perso dentro un kernel a cicli esatti sposta il pennello.
  // La lettura della risposta cade in overscan (riga 247), quindi tutta
  // l'attesa utile sta fuori dal disegno.
  input wire arm_run_now,
  output wire ds_wait,
  // loader
  input wire rom_load_we, input wire [16:0] rom_load_addr, input wire [7:0] rom_load_data,
  // Dimensione REALE della cartuccia caricata, in byte. Stella confronta il
  // campione audio digitale con `myImage.size()` (CartCDF.cxx:287), non con la
  // capacita' della memoria: da quando ROM_DEPTH e' 131072 per tutti, usarlo
  // come limite fa leggere M10K vuota invece di cadere nel ramo RAM/zero per
  // le cartucce piu' piccole (Tutankham e' 32 KB). Serve la taglia vera.
  input wire [18:0] rom_bytes,
  // debug
  output wire [3:0] dbg_bank, output wire [7:0] dbg_mode,
  output wire        dbg_lda_valid, output wire [12:0] dbg_lda_addr, output wire [7:0] dbg_ff_val
);
  localparam [7:0] COMMSTREAM = 8'd32, JUMPSTREAM_BASE = 8'd33;

  wire [15:0] ds_base, ds_inc_base, wf_base, prog_off;
  wire [7:0] amp_stream, jump_mask; wire is_plus; wire [2:0] start_bank;
  wire [31:0] cb0_nc, cb1_nc, cb2_nc, cb3_nc;
  cdf_family_params params(.family_sel(family_sel), .ds_base(ds_base),
    .ds_inc_base(ds_inc_base), .wf_base(wf_base), .amp_stream(amp_stream),
    .jump_mask(jump_mask), .is_plus(is_plus), .start_bank(start_bank),
    .prog_off(prog_off),
    .cb0(cb0_nc), .cb1(cb1_nc), .cb2(cb2_nc), .cb3(cb3_nc));

  //--------------------------------------------------------------------------
  // Memories (Livello 1 - true dual-clock).
  //   Porta A = ARM (clk_vid): fetch/dati diretti, nessun handshake CDC.
  //   Porta B = clk_sys: engine 6502 (read/write) + loader (write, durante il
  //             download, quando il 6502 e' in reset - mutuamente esclusivi).
  //--------------------------------------------------------------------------
  wire [31:0] rom_q_a, rom_q_a_next, rom_q_b;
  reg  [14:0] rom_addr_b;
  // Porta A ROM: sola lettura ARM (l'ARM non scrive mai region 0).
  wire [14:0] rom_addr_a = arm_addr_eff[16:2];
  // PASSO 15: la parola SUCCESSIVA, senza sommatori in serie. Sul
  // ramo anticipato arriva gia' sommata dal core; su quello
  // registrato la somma parte da un registro, quindi non e' in
  // cascata al sommatore dell'indirizzo.
  wire [31:0] arm_addr_eff_nx = bus_req_pre ? bus_addr_pre_nx
                                            : (bus_addr + 32'd4);
  wire [14:0] rom_addr_a_nx = arm_addr_eff_nx[16:2];
  // Porta B ROM: engine legge da rom_addr_b; durante il download il loader
  // scrive qui (il byte va nella lane selezionata dai due bit bassi).
  wire        rom_wren_b  = rom_load_we;
  wire [14:0] rom_addr_bp = rom_load_we ? rom_load_addr[16:2] : rom_addr_b;
  wire [31:0] rom_data_b  = {4{rom_load_data}};
  wire [3:0]  rom_be_b    = 4'b0001 << rom_load_addr[1:0];

  cdf_rom_m10k #(.INIT_FILE(ROM_FILE)) rom_i (
    .clk_a(clk_vid), .addr_a(rom_addr_a), .addr_a_nx(rom_addr_a_nx),
    .q_a(rom_q_a), .q_a_next(rom_q_a_next),
    .clk_b(clk),     .addr_b(rom_addr_bp), .data_b(rom_data_b),
    .byteena_b(rom_be_b), .wren_b(rom_wren_b), .q_b(rom_q_b)
  );

  // Harmony RAM 8KB. During download the first 2KB receive the driver image,
  // the rest is zero-filled (Stella setInitialState()). The display image IS
  // this RAM from +2KB up - there is no separate display memory in CDF.
  localparam [16:0] HARMONY_LOAD_BYTES   = 17'd32768;
  localparam [16:0] HARMONY_DRIVER_BYTES = 17'd2048;
  wire [31:0] harmony_q_a, harmony_q_b;
  reg  [12:0] harmony_addr_b; reg harmony_we_b; reg [3:0] harmony_be_b;
  reg  [31:0] harmony_wdata_b;
  wire harmony_load_we = rom_load_we && (rom_load_addr < HARMONY_LOAD_BYTES);
  wire [7:0] harmony_load_data = (rom_load_addr < HARMONY_DRIVER_BYTES) ? rom_load_data : 8'd0;

  // Livello 3 - rimappatura ping-pong: una word 11-bit (0..2047) che cade nella
  // Display Data (512..1535) viene traslata di +2048 quando il banco selezionato
  // e' 1, portandola nel buffer 1 (2560..3583). Le word fuori dalla Display Data
  // (driver/pointer/C-vars) restano invariate: NON duplicate.
  // 2 ago 2026: con la RAM a 32 KB (8192 word) TUTTO lo spazio serve ai dati,
  // quindi non esiste piu' un secondo buffer a +2048 word. `disp_bank` e'
  // comunque inchiodato a 0 (atari2600_arm_subsystem.sv:306/313) da quando il
  // Livello 3 e' stato disattivato, percio' la funzione era gia' di fatto un
  // passa-attraverso. Resta come punto unico da cui ripartire se il ping-pong
  // verra' ripreso: servira' AW=14 (64 KB) per ospitare il banco 1.
  function [12:0] remap_disp; input [12:0] w; input bank; begin
    remap_disp = w; // bank ignorato: nessun secondo buffer a 32 KB
  end endfunction

  // Porta A Harmony = ARM (clk_vid).
  // Ping-pong ATTIVO: l'ARM scriverebbe il buffer BACK = ~disp_bank.
  // Ping-pong DISATTIVO (stato attuale, PINGPONG_ACTIVE=0): l'ARM usa lo STESSO
  // banco della porta B (disp_bank), cosi' con disp_bank=0 entrambe le porte
  // lavorano sul buffer 0 e il comportamento e' identico al Livello 2 (nessun
  // disaccoppiamento). Vedi la nota nel subsystem e FASE8_livello3_pingpong.md.
  localparam PINGPONG_ACTIVE = 1'b0;
  // STADIO 2: quando il core annuncia l'accesso (bus_req_pre), si presenta
  // gia' il suo indirizzo: la M10K lo cattura un ciclo prima e il dato e'
  // pronto al primo ciclo di S_MEM.
  wire [31:0] arm_addr_eff = bus_req_pre ? bus_addr_pre : bus_addr;
  wire [12:0] harmony_wa_a  = arm_addr_eff[14:2];
  wire        arm_bank      = PINGPONG_ACTIVE ? ~disp_bank : disp_bank;
  wire [12:0] harmony_addr_a = remap_disp(harmony_wa_a, arm_bank);
  wire [31:0] harmony_data_a = bus_wdata;
  wire [3:0]  harmony_be_a   = bus_be;
  wire        harmony_we_a   = bus_req && bus_we && (bus_addr[31:28]==4'h4);
  // Porta B Harmony = clk_sys: engine 6502 legge/scrive il buffer FRONT =
  // disp_bank; muxato col loader durante il download (loader vince, banco 0 -
  // il download riempie sempre il buffer 0, che sara' il primo front).
  wire [12:0] harmony_addr_bp = harmony_load_we ? rom_load_addr[14:2]
                                                : remap_disp(harmony_addr_b, disp_bank);
  wire [31:0] harmony_data_bp = harmony_load_we ? {4{harmony_load_data}} : harmony_wdata_b;
  wire [3:0]  harmony_be_bp   = harmony_load_we ? (4'b0001 << rom_load_addr[1:0]) : harmony_be_b;
  wire        harmony_we_bp   = harmony_load_we | harmony_we_b;
  harmony_m10k_tdp #(.INIT_FILE(RAM_FILE), .AW(13)) harmony_ram_i (
    .clk_a(clk_vid), .addr_a(harmony_addr_a), .data_a(harmony_data_a),
    .byteena_a(harmony_be_a), .wren_a(harmony_we_a), .q_a(harmony_q_a),
    .clk_b(clk), .addr_b(harmony_addr_bp), .data_b(harmony_data_bp),
    .byteena_b(harmony_be_bp), .wren_b(harmony_we_bp), .q_b(harmony_q_b)
  );

  //--------------------------------------------------------------------------
  // State
  //--------------------------------------------------------------------------
  reg [3:0]  bank_reg;
  reg [7:0]  mode_reg;                    // Stella resets to 0xFF: fast fetch OFF
  reg        lda_valid;  reg [12:0] lda_addr;
  reg [1:0]  fj_active;  reg [12:0] fj_operand; reg [7:0] fj_stream;
  reg [7:0]  amp_cached;
  reg [31:0] music_freq [0:2];
  reg [31:0] music_cnt  [0:2];
  reg [7:0]  wave_size  [0:2];            // Stella default 27
  reg [7:0]  ff_val;                      // cached RAM[ff_offset]
  reg [9:0]  mus_div;

  wire fast_fetch_en    = (mode_reg[3:0] == 4'h0);
  wire digital_audio_en = (mode_reg[7:4] == 4'h0);

  assign dbg_bank = bank_reg;
  assign dbg_mode = mode_reg;
  // See the note above ff_hit/ff_norm: these three exist to give the
  // toolchain a real (used) reader for lda_valid/lda_addr/ff_val.
  assign dbg_lda_valid = lda_valid;
  assign dbg_lda_addr  = lda_addr;
  assign dbg_ff_val    = ff_val;

  // one trigger per CPU cycle: cartridge address changed OR bus direction
  // changed at the same address.
  //
  // BUG CORRETTO (1 agosto 2026), stesso difetto trovato e provato su
  // dpcplus_bridge (vedi la nota estesa in rtl/dpcplus_bridge.sv). Su
  //     STA abs,X / STA abs,Y / STA (zp),Y
  // senza attraversamento di pagina il 6502 emette un ciclo di dummy-READ e
  // poi il ciclo di SCRITTURA allo STESSO indirizzo: con il solo confronto
  // sull'indirizzo la scrittura non generava trigger e veniva scartata.
  // Il confronto sul verso (rwn) la recupera e non puo' scattare durante uno
  // stallo RDY, dove indirizzo e verso restano entrambi fermi.
  // Su Draconian (regressione d'oro CDF) non cambia nulla: MISURATO 0
  // scritture con indirizzo ripetuto.
  wire m6502_cs = (m6502_addr[15:12] == 4'h1);
  reg [15:0] addr_d;
  reg        rwn_d;
  wire trigger = m6502_cs && ((m6502_addr != addr_d) || (m6502_rwn != rwn_d));
  wire [11:0] off_in = m6502_addr[11:0];

  reg [11:0] off; reg [7:0] din;

  //--------------------------------------------------------------------------
  // helpers
  //--------------------------------------------------------------------------
  function [14:0] prog_byte_addr; input [12:0] o13; begin
    prog_byte_addr = {2'd0, prog_off[12:0]} + {bank_reg[2:0], 12'd0} + {2'd0, o13};
  end endfunction
  function [7:0] lane8; input [31:0] w; input [1:0] sel; begin
    lane8 = (sel==2'd0) ? w[7:0] : (sel==2'd1) ? w[15:8] : (sel==2'd2) ? w[23:16] : w[31:24];
  end endfunction
  function [12:0] ds_ptr_word; input [7:0] stream; begin
    ds_ptr_word = ds_base[14:2] + {5'd0, stream};
  end endfunction
  function [12:0] ds_inc_word; input [7:0] stream; begin
    ds_inc_word = ds_inc_base[14:2] + {5'd0, stream};
  end endfunction
  // display byte address (Harmony RAM at +2KB) selected by a datastream ptr
  function [14:0] disp_byte_addr; input [31:0] ptr; reg [15:0] idx; begin
    idx = is_plus ? ptr[31:16] : {4'd0, ptr[31:20]};
    disp_byte_addr = 15'd2048 + idx[14:0];
  end endfunction
  function disp_ok; input [31:0] ptr; begin
    disp_ok = is_plus ? (ptr[31:16] < (RAM_DEPTH-2048)) : 1'b1;
  end endfunction
  // getWaveform(): display byte address of the current waveform sample
  function [14:0] wf_disp_byte; input [31:0] wfp; input [31:0] cnt; input [7:0] sz;
    reg [31:0] offw; reg [31:0] idx; begin
    offw = wfp - 32'h40000800;
    if (!is_plus && offw >= 32'd4096) offw = offw & 32'd4095;
    if (is_plus && offw >= (RAM_DEPTH-2048)) offw = 32'd0;
    idx = offw + (cnt >> sz[4:0]);
    wf_disp_byte = 15'd2048 + idx[14:0]; // wraps inside the 32KB RAM
  end endfunction

  //--------------------------------------------------------------------------
  // 6502-side engine FSM (6-bit state)
  //--------------------------------------------------------------------------
  // ##########################################################################
  // REGOLA DI TEMPORIZZAZIONE DELLA PORTA B - leggere PRIMA di toccare la FSM
  // ##########################################################################
  // Dal 7 agosto 2026 la porta B delle M10K non ha piu' il registro di uscita
  // (outdata_reg_b UNREGISTERED in harmony_m10k_tdp.sv e cdf_m10k_memories.sv).
  // Resta il solo registro di INDIRIZZO, quindi:
  //
  //     q_b nel ciclo C  =  mem[ addr_b nel ciclo C-1 ]
  //                      =  mem[ valore assegnato ad addr_b nello stato C-2 ]
  //
  //   ==> UN INDIRIZZO ASSEGNATO NELLO STATO S SI LEGGE NELLO STATO S+2.
  //       (prima della modifica erano S+3: uno stadio di registro in piu')
  //
  // Il dato e' valido SOLO in quel ciclo, e sparisce appena l'indirizzo cambia:
  // non si puo' piu' "arrivare tardi" alla lettura. Questa e' esattamente la
  // trappola in cui sono cascato il 6 agosto adeguando la memoria e UN SOLO
  // consumatore su quattro. I consumatori della porta B sono, per intero:
  //
  //   1. pipeline di lettura ROM        trigger -> S_R1 -> S_DEC
  //   2. selezione banco pari/dispari   sel_b_d1 in cdf_m10k_memories.sv
  //   3. serve datastream               S_DS0, S_DS2, S_DS3, S_DS6
  //   4. sonda 0x4C                     S_JP0, S_JP2, S_JP3 (+ S_DS4, S_DS6)
  //   5. DSWRITE/DSPTR                  S_CW0, S_CW2, S_CW3
  //   6. refresh di ff_val              S_FF0, S_FF2
  //   7. catena audio                   S_BG0..S_BG7 e S_BGD0..S_BGD3
  //   8. percorso del loader            SOLE SCRITTURE: indata/wraddress/byteena
  //                                     restano registrati, quindi immutato
  //
  // Gli stati S_R2, S_DS1, S_DS5, S_JP1, S_CW1, S_FF1, S_BG8, S_BG9, S_BGD2
  // erano stati di puro ritardo e non sono piu' raggiunti; restano nella case
  // come rete di sicurezza (e per non rinumerare le sonde del testbench).
  //
  // BILANCIO DEL CICLO 6507: fra due accessi ci sono 12 clk_sys (misurato).
  // Stati percorsi DOPO il ciclo di trigger, fino a m6502_dout pronto:
  //   prima:  S_R1,S_R2,S_DEC + S_DS0,DS1,DS2,DS3,DS4,DS6 = 9
  //   ora:    S_R1,S_DEC      + S_DS0,DS2,DS3,DS6         = 6
  // Tre cicli in meno, tutti restituiti al margine. E' il margine che decide
  // se un'attesa breve del datastream (S_DWAIT) si conclude o viene troncata
  // dal trigger del ciclo dopo - il difetto di Lode Runner. Quanto scendano
  // davvero i serve troncati lo dice la sonda [abort], non questo conto.
  // ##########################################################################
  localparam [5:0]
    S_IDLE = 6'd0,
    S_R1   = 6'd1,  S_R2  = 6'd2,  S_DEC  = 6'd3,
    S_DS0  = 6'd4,  S_DS1 = 6'd5,  S_DS2  = 6'd6,  S_DS3 = 6'd7,
    S_DS4  = 6'd8,  S_DS5 = 6'd9,  S_DS6  = 6'd10,
    S_JP0  = 6'd11, S_JP1 = 6'd12, S_JP2  = 6'd13, S_JP3 = 6'd14,
    S_CW0  = 6'd15, S_CW1 = 6'd16, S_CW2  = 6'd17, S_CW3 = 6'd18,
    S_FF0  = 6'd19, S_FF1 = 6'd20, S_FF2  = 6'd21, S_FF3 = 6'd22,
    S_BG0  = 6'd23, S_BG1 = 6'd24, S_BG2  = 6'd25, S_BG3 = 6'd26,
    S_BG4  = 6'd27, S_BG5 = 6'd28, S_BG6  = 6'd29, S_BG7 = 6'd30,
    S_BG8  = 6'd31, S_BG9 = 6'd32,
    S_BGD0 = 6'd33, S_BGD1 = 6'd34, S_BGD2 = 6'd35, S_BGD3 = 6'd36,
    // FASE9: stato di attesa PRIMA del serve datastream. Trattiene il serve
    // (ri-presentando l'indirizzo del puntatore) finche' l'ARM non ha finito,
    // cosi' il puntatore viene letto DOPO l'aggiornamento dell'ARM.
    S_DWAIT = 6'd37;

  reg [5:0] state;
  // PASSO ARSP: ritenuta del 6507 PARALLELA alla macchina a stati.
  // Non e' uno stato della FSM perche' `trigger` prelaziona qualunque
  // stato al primo accesso del 6507, e il ciclo dopo la scrittura di DSPTR
  // e' un'altra SCRITTURA (il 6502 onora RDY solo sulle letture): l'attesa
  // verrebbe abortita subito e la scrittura di ritorno del puntatore
  // (S_CW3) andrebbe PERDUTA. Provato: Spiders non entrava piu' in partita.
  reg        arsp_hold;
  reg [17:0] arsp_cnt;

  // FASE9: il 6507 e' in attesa esattamente quando la FSM e' parcheggiata in
  // S_DWAIT. Nessun latch: e' una funzione dello stato corrente.
  assign ds_wait = (state == S_DWAIT) || arsp_hold;

  // contatore di timeout (§3.3): azzerato all'ingresso in S_DWAIT
  reg [17:0] dwait_cnt;
  reg serving_fj;               // current DS serve is a fastjump operand
  // 7 ago 2026: `ds_extra_wait` (serve lanciato da S_JP3, con la lettura della
  // display word in ritardo di uno) e' STATO TOLTO. Con la porta B a 1 ciclo il
  // percorso S_JP3 ha il suo stato dedicato di riscrittura (S_DS4) e quello del
  // serve normale ha il suo (S_DS3): non serve piu' un flag per allungare la
  // catena di uno, e lo stato di riempimento S_DS5 e' sparito con lui.
  reg [7:0] serve_stream;
  reg [7:0] rb_lat, b1_lat;
  reg [31:0] ptr_lat, inc_lat;
  reg is_dsptr;
  reg [31:0] wf_lat [0:2];
  reg [7:0] samp_lat [0:1];
  reg [31:0] dig_addr;

  // blocking scratch
  reg [7:0] rbv, normv, sv;
  reg [14:0] dba;
  reg hitv, jpmatch;

  task do_bank; input [11:0] o; begin
    case (o)
      12'hff4: bank_reg <= is_plus ? 4'd0 : 4'd6;
      12'hff5: bank_reg <= is_plus ? 4'd1 : 4'd0;
      12'hff6: bank_reg <= is_plus ? 4'd2 : 4'd1;
      12'hff7: bank_reg <= is_plus ? 4'd3 : 4'd2;
      12'hff8: bank_reg <= is_plus ? 4'd4 : 4'd3;
      12'hff9: bank_reg <= is_plus ? 4'd5 : 4'd4;
      12'hffa: bank_reg <= is_plus ? 4'd6 : 4'd5;
      12'hffb: bank_reg <= is_plus ? 4'd0 : 4'd6;
      default: ;
    endcase
  end endtask

  // fast-fetch operand hit test / normalization (Stella: peekvalue window).
  // Kept as `function` (verified in end-to-end simulation across multiple
  // ARM runs): an earlier attempt to replace this with always-on
  // combinational wires changed the evaluation order enough to desync the
  // ARM run / 6502 protocol after a few CALLFN cycles (regression only
  // visible after run #2, caught by re-running the sim - do not "simplify"
  // this back to plain top-level wires without re-running the sim for
  // several ARM runs, not just the first one).
  //
  // The Quartus 17.0.2 warning (10036) on lda_valid/lda_addr/ff_val is a
  // known linter false positive: those regs are read only inside a function
  // body with positional inputs, which its unused-signal pass doesn't trace.
  // Silenced without touching the read (and therefore without touching
  // timing) by also driving them out on unused debug ports, which forces
  // the synthesis tool to treat them as used regardless of the function-call
  // tracing gap.
  // 4 agosto 2026: la somma del limite superiore era valutata a 8 BIT.
  // Stella la calcola in int (CartCDF.cxx:268):
  //     peekvalue <= myRAM[myFastFetcherOffset] + myAmplitudeStream
  // quindi non tronca mai. Da noi ff_val e amp_stream sono entrambi [7:0] e
  // Verilog dimensiona l'espressione sull'operando piu' largo: con
  // ff_val+amp_stream oltre 255 la somma si ribaltava e la finestra di fast
  // fetch si chiudeva, facendo arrivare al 6507 byte grezzi di ROM invece del
  // datastream. Misurato: ff_val max osservato 200 e amp_stream 35 (=235), cioe'
  // il difetto oggi NON scatta su nessuna ROM provata - ma il margine e' 20.
  // Allargato a 9 bit, che e' il minimo che rende impossibile il riporto.
  function ff_hit; input [7:0] b; begin
    ff_hit = fast_fetch_en && lda_valid && ({1'b0,off} == lda_addr) &&
             ((ff_offset != 16'd0) ? ((b >= ff_val) &&
                                      ({1'b0,b} <= ({1'b0,ff_val} + {1'b0,amp_stream})))
                                   : (b <= amp_stream));
  end endfunction
  function [7:0] ff_norm; input [7:0] b; begin
    ff_norm = (ff_offset != 16'd0) ? (b - ff_val) : b;
  end endfunction

  // Stella peek() tail: steps 4-6 (clear pending, hotspots, LDA# detect)
  task peek_tail; input [7:0] b; begin
    lda_valid <= 1'b0;
    fj_operand <= 13'd0;   // Stella: myJMPoperandAddress = 0
    do_bank(off);
    if (fast_fetch_en && (b == 8'hA9 ||
                          (ldx_en && b == 8'hA2) ||
                          (ldy_en && b == 8'hA0))) begin
      lda_valid <= 1'b1;
      lda_addr <= {1'b0,off} + 13'd1;
    end
    m6502_dout <= b;
  end endtask

  wire [14:0] pba_in = prog_byte_addr({1'b0, off_in});
  wire [14:0] pba_cur = prog_byte_addr({1'b0, off});
  wire [14:0] pba_p1 = prog_byte_addr({1'b0, off} + 13'd1);
  wire [14:0] pba_p2 = prog_byte_addr({1'b0, off} + 13'd2);

  integer k;
  always @(posedge clk) begin
    callfn <= 1'b0;
    harmony_we_b <= 1'b0;
    if (rst) begin
      state <= S_IDLE; addr_d <= 16'hFFFF; rwn_d <= 1'b1; dwait_cnt <= 18'd0;
      bank_reg <= {1'b0, start_bank}; mode_reg <= 8'hFF;
      lda_valid <= 0; lda_addr <= 0; fj_active <= 0; fj_operand <= 0; fj_stream <= 0;
      amp_cached <= 0; m6502_dout <= 0; mus_div <= 0; ff_val <= 0;
      off <= 0; din <= 0; serving_fj <= 0; serve_stream <= 0;
      rb_lat <= 0; b1_lat <= 0; ptr_lat <= 0; inc_lat <= 0; is_dsptr <= 0;
      dig_addr <= 0;
      rom_addr_b <= 0; harmony_addr_b <= 0; harmony_be_b <= 0; harmony_wdata_b <= 0;
      callfn_val <= 0;
      arsp_hold <= 1'b0; arsp_cnt <= 18'd0;
      cb_ack <= 0; cb_ret <= 0;
      for (k = 0; k < 3; k = k + 1) begin
        music_freq[k] <= 32'd0; music_cnt[k] <= 32'd0; wave_size[k] <= 8'd27;
        wf_lat[k] <= 32'd0;
      end
      samp_lat[0] <= 0; samp_lat[1] <= 0;
    end else begin
      addr_d <= m6502_addr;
      rwn_d  <= m6502_rwn;

      // 20 kHz music clock (Stella updateMusicModeDataFetchers)
      if (mus_div == 10'd715) begin
        mus_div <= 0;
        for (k = 0; k < 3; k = k + 1) music_cnt[k] <= music_cnt[k] + music_freq[k];
      end else mus_div <= mus_div + 1'd1;

      // music callbacks from the ARM trap (after the tick so ResetWave wins)
      if (cb_req && !cb_ack) begin
        cb_ack <= 1'b1;
        cb_ret <= 32'd0;
        if (cb_v1 < 3) begin
          case (cb_id)
            2'd0: music_freq[cb_v1[1:0]] <= cb_v2;
            2'd1: music_cnt[cb_v1[1:0]]  <= 32'd0;
            2'd2: cb_ret <= music_cnt[cb_v1[1:0]];
            2'd3: wave_size[cb_v1[1:0]]  <= cb_v2[7:0];
          endcase
        end
      end else if (!cb_req) cb_ack <= 1'b0;

      //----------------------------------------------------------------
      // PASSO ARSP: la ritenuta si arma sulla scrittura di DSPTR ($FF1)
      // fatta mentre una corsa ARM e' in volo fuori dal disegno, e cade da
      // sola quando la corsa finisce (o al tetto ARSP_BUDGET, rete
      // anti-deadlock: un 6507 fermo non puo' far avanzare niente).
      //----------------------------------------------------------------
      if (!arsp_hold) begin
        arsp_cnt <= 18'd0;
        if (ARSP_EN && (is_plus || !ARSP_PLUS_ONLY) &&
            trigger && !m6502_rwn && (off_in == 12'hff1) && arm_run_now)
          arsp_hold <= 1'b1;
      end else begin
        arsp_cnt <= arsp_cnt + 1'b1;
        if (!arm_run_now || arsp_cnt >= ARSP_BUDGET) arsp_hold <= 1'b0;
      end

      //----------------------------------------------------------------
      // per-CPU-cycle trigger (preempts any background work)
      //----------------------------------------------------------------
      if (trigger) begin
        off <= off_in; din <= m6502_din;
        if (m6502_rwn) begin
          rom_addr_b <= pba_in[14:2];
          state <= S_R1;
        end else begin
          state <= S_IDLE;
          case (off_in)
            12'hff0: begin
              harmony_addr_b <= ds_ptr_word(COMMSTREAM);
              is_dsptr <= 1'b0; state <= S_CW0;
            end
            12'hff1: begin
              harmony_addr_b <= ds_ptr_word(COMMSTREAM);
              is_dsptr <= 1'b1; state <= S_CW0;
            end
            12'hff2: mode_reg <= m6502_din;
            12'hff3: begin callfn <= 1'b1; callfn_val <= m6502_din; end
            default: do_bank(off_in);
          endcase
        end
      end else begin
        case (state)
          //------------------------------------------------------------
          S_IDLE: begin
            if (ff_offset != 16'd0) begin
              harmony_addr_b <= ff_offset[14:2];
              state <= S_FF0;
            end else state <= S_BG0;
          end

          //------------------------------------------------------------
          // read pipeline: indirizzo emesso al trigger, byte valido in S_DEC.
          // Con la porta B a 1 ciclo bastano DUE stati (S+2): S_R2 non serve.
          S_R1: state <= S_DEC;
          S_R2: state <= S_DEC;   // non piu' raggiunto (rete di sicurezza)
          S_DEC: begin
            rbv = lane8(rom_q_b, pba_cur[1:0]);
            rb_lat <= rbv;
            if (fj_active != 2'd0 && {1'b0,off} == fj_operand && fj_operand != 13'd0) begin
              // 1) serve FASTJMP operand
              serving_fj <= 1'b1;
              serve_stream <= fj_stream;
              fj_active <= fj_active - 1'd1;
              fj_operand <= fj_operand + 1'd1;
              harmony_addr_b <= ds_ptr_word(fj_stream);
              state <= S_DS0;
            end else if (fast_fetch_en && rbv == 8'h4C) begin
              // 2) probe JMP $0000; speculatively read ptr/inc of the would-be
              // datastream (relevant only with ff_offset remapping)
              rom_addr_b <= pba_p1[14:2];
              harmony_addr_b <= ds_ptr_word(ff_norm(rbv));
              m6502_dout <= rbv;
              state <= S_JP0;
            end else if (ff_hit(rbv)) begin
              // 3) LDA# fast fetch
              lda_valid <= 1'b0;
              normv = ff_norm(rbv);
              if (normv == amp_stream) begin
                m6502_dout <= amp_cached;
                state <= S_IDLE;
              end else begin
                serving_fj <= 1'b0;
                serve_stream <= normv;
                harmony_addr_b <= ds_ptr_word(normv);
                // FASE9 (attesa selettiva) DISATTIVATA il 5 agosto 2026.
                //
                // Qui il serve veniva parcheggiato in S_DWAIT quando l'ARM era
                // ancora in corsa, per non leggere un puntatore che l'ARM non
                // aveva ancora aggiornato. MISURATO che quell'attesa non si
                // conclude MAI da sola: `trigger` (riga 428) fa ripartire la
                // macchina a stati all'inizio di ogni accesso del 6507, e
                // l'attesa contava di fermarlo abbassando RDY - che pero' viene
                // campionato una volta per ciclo CPU e arriva troppo tardi.
                //
                // Bilancio del ciclo, misurato: fra due accessi del 6507 ci
                // sono 12 clk_sys; il serve ne consuma 3 (S_R1..S_DEC) + 7
                // (S_DS0..S_DS6) = 10. Basta, ma solo senza attesa. Con
                // l'attesa il serve viene TRONCATO e `m6502_dout` resta al
                // valore precedente: il 6507 rilegge il byte di prima.
                //
                // Conteggio su 20 frame, bridge originale:
                //   Elevator Agent  114 troncati, 0 attese concluse da sole
                //   Tutankham         1 troncato
                //   Spiders           0
                //   Draconian         1 troncato (corsa d'oro, 200k cicli)
                // In NESSUN titolo l'attesa e' mai arrivata in fondo: non ha
                // mai prodotto il beneficio per cui era stata scritta, e ha
                // solo consegnato byte sbagliati.
                //
                // Su Elevator Agent e' la causa del tono fisso a 15700 Hz: il
                // gioco riposiziona il puntatore di COMMSTREAM a 7 e legge 8
                // byte (7..14) per prendere i valori audio che l'ARM ha appena
                // scritto ai byte 13,14,15. Con quattro letture troncate il
                // puntatore arriva solo a 11, i valori audio non vengono mai
                // letti, e il buffer in pagina zero $91-$98 resta pieno di
                // a9/ff che finiscono tali e quali su AUDF/AUDC/AUDV.
                //
                // La protezione vera contro la corsa ARM/6507 sulla Display
                // Data resta il ping-pong (Livello 3), non un'attesa che il
                // 6507 non puo' rispettare.
                //
                // SCELTA CONSERVATIVA: l'attesa viene tolta SOLO per CDFJ+.
                // Il difetto e' generale, ma su CDF si presenta una volta ogni
                // 200.000 cicli (Draconian) e togliendola li' la traccia
                // cambia (ds_serves 2080->3915, tia_writes 6527->8487): il
                // gioco reagisce al dato ora corretto e imbocca un'altra
                // strada. Non e' detto sia un peggioramento, ma i 21 titoli
                // CDF collaudati su hardware valgono piu' di quel singolo
                // evento, e il confronto visivo disponibile (2 fotogrammi) non
                // basta a dichiararli salvi. Il ramo CDF va rivisto quando si
                // potra' confrontare l'uscita video titolo per titolo.
                dwait_cnt <= 18'd0;
                state <= (is_plus || !ds_wait_arm) ? S_DS0 : S_DWAIT;
              end
            end else begin
              peek_tail(rbv);   // steps 4-7
              state <= S_IDLE;
            end
          end

          //------------------------------------------------------------
          // FASE9 - attesa selettiva: trattiene il serve datastream finche'
          // l'ARM non ha finito. Ri-presenta l'indirizzo del puntatore ad ogni
          // ciclo cosi' che, all'uscita, S_DS0 trovi il dato AGGIORNATO (la
          // latenza della M10K e' gia' assorbita). ds_wait (= state==S_DWAIT)
          // toglie RDY al 6507 per tutta la durata dell'attesa.
          // Uscita: (a) l'ARM ha finito o il Kernel e' terminato (ds_wait_arm
          // cade da solo - nessun latch), oppure (b) TIMEOUT §3.3, rete di
          // sicurezza contro il deadlock (6507 fermo non puo' scrivere VBLANK).
          S_DWAIT: begin
            harmony_addr_b <= ds_ptr_word(serve_stream);
            dwait_cnt <= dwait_cnt + 1'b1;
            // ATTESA A BUDGET FINITO (7 ago 2026, esperimento).
            // Prima l'attesa finiva solo quando l'ARM finiva: MISURATO che non
            // succede MAI (zero uscite naturali su ogni titolo), quindi il
            // trigger del ciclo dopo la troncava e il serve non partiva -
            // m6502_dout restava al byte precedente (su Lode Runner: a9).
            // Ora si aspetta finche' c'e' margine dentro il ciclo del 6507, poi
            // SI SERVE COMUNQUE: il puntatore puo' essere vecchio di un ciclo,
            // ma un byte del datastream e' comunque piu' plausibile di una
            // ripetizione dell'opcode precedente. WAIT_MAX resta la rete
            // anti-deadlock, DWAIT_BUDGET e' il margine utile del ciclo.
            if (!ds_wait_arm || dwait_cnt >= DWAIT_BUDGET ||
                dwait_cnt >= WAIT_MAX)                 state <= S_DS0;
            else                                       state <= S_DWAIT;
          end

          //------------------------------------------------------------
          // datastream / fastjump serve. Entered from S_DEC (ptr address
          // already issued) or from S_JP3 (ptr/inc already latched).
          // Indirizzo del puntatore emesso in S_DEC (o ri-emesso in S_DWAIT):
          // si legge due stati dopo, cioe' in S_DS2.
          S_DS0: begin
            harmony_addr_b <= ds_inc_word(serve_stream);  // letto in S_DS3
            state <= S_DS2;
          end
          S_DS1: state <= S_DS2;   // non piu' raggiunto (rete di sicurezza)
          S_DS2: begin // harmony_q_b = pointer
            ptr_lat <= harmony_q_b;
            dba = disp_byte_addr(harmony_q_b);
            harmony_addr_b <= dba[14:2];                  // letto in S_DS6
            state <= S_DS3;
          end
          // S_DS3 fa DUE cose che prima erano su due stati: legge l'incremento
          // e nello stesso ciclo emette la riscrittura del puntatore. Puo'
          // farlo perche' l'incremento e' gia' su harmony_q_b: usarlo qui e'
          // identico a rileggere inc_lat il ciclo dopo, senza costare un ciclo.
          // L'indirizzo del puntatore emesso qui NON disturba la lettura della
          // display word: quella e' gia' "in volo" (indirizzo emesso in S_DS2)
          // e arriva in S_DS6, dove la scrittura viene presentata alla memoria.
          S_DS3: begin // harmony_q_b = increment
            inc_lat <= harmony_q_b;
            harmony_addr_b <= ds_ptr_word(serve_stream);
            harmony_wdata_b <= serving_fj
              ? (ptr_lat + (is_plus ? 32'h00010000 : 32'h00100000))
              : (ptr_lat + (is_plus ? {harmony_q_b[23:0], 8'd0}
                                    : {harmony_q_b[19:0], 12'd0}));
            harmony_be_b <= 4'b1111; harmony_we_b <= 1'b1;
            state <= S_DS6;
          end
          // S_DS4 serve ORA al solo percorso S_JP3, dove ptr_lat e inc_lat sono
          // gia' latchati (letture speculative) e l'indirizzo della display word
          // e' stato emesso in S_JP3: si legge in S_DS6, due stati dopo.
          S_DS4: begin // write back incremented pointer (percorso S_JP3)
            harmony_addr_b <= ds_ptr_word(serve_stream);
            harmony_wdata_b <= serving_fj
              ? (ptr_lat + (is_plus ? 32'h00010000 : 32'h00100000))
              : (ptr_lat + (is_plus ? {inc_lat[23:0], 8'd0} : {inc_lat[19:0], 12'd0}));
            harmony_be_b <= 4'b1111; harmony_we_b <= 1'b1;
            state <= S_DS6;
          end
          S_DS5: state <= S_DS6;   // non piu' raggiunto (rete di sicurezza)
          S_DS6: begin // harmony_q_b = display word
            dba = disp_byte_addr(ptr_lat);
            m6502_dout <= disp_ok(ptr_lat) ? lane8(harmony_q_b, dba[1:0]) : 8'd0;
            state <= S_IDLE;
          end

          //------------------------------------------------------------
          // 0x4C probe (with speculative ptr/inc reads on the RAM port)
          // S_DEC ha emesso rom=pba_p1 e harmony=ptr_word: si leggono in S_JP2.
          // Qui si emettono rom=pba_p2 e harmony=inc_word: si leggono in S_JP3.
          S_JP0: begin
            rom_addr_b <= pba_p2[14:2];
            harmony_addr_b <= ds_inc_word(ff_norm(rb_lat));
            state <= S_JP2;
          end
          S_JP1: state <= S_JP2;   // non piu' raggiunto (rete di sicurezza)
          S_JP2: begin
            b1_lat <= lane8(rom_q_b, pba_p1[1:0]);
            ptr_lat <= harmony_q_b;       // speculative pointer
            state <= S_JP3;
          end
          S_JP3: begin
            jpmatch = ((b1_lat & jump_mask) == 8'd0) &&
                      (lane8(rom_q_b, pba_p2[1:0]) == 8'd0);
            if (jpmatch) begin
              fj_active <= 2'd2;
              fj_operand <= {1'b0,off} + 13'd1;
              fj_stream <= b1_lat + JUMPSTREAM_BASE;
              // dout already holds the 0x4C byte; Stella returns here
              state <= S_IDLE;
            end else if (ff_hit(rb_lat)) begin
              // 0x4C as LDA# operand (only reachable with ff_offset != 0)
              lda_valid <= 1'b0;
              normv = ff_norm(rb_lat);
              if (normv == amp_stream) begin
                m6502_dout <= amp_cached;
                state <= S_IDLE;
              end else begin
                inc_lat <= harmony_q_b;    // speculative increment
                dba = disp_byte_addr(ptr_lat);
                harmony_addr_b <= dba[14:2];
                serving_fj <= 1'b0;
                serve_stream <= normv;
                state <= S_DS4;
              end
            end else begin
              peek_tail(rb_lat);
              state <= S_IDLE;
            end
          end

          //------------------------------------------------------------
          // DSWRITE ($FF0) / DSPTR ($FF1)
          // indirizzo del puntatore COMMSTREAM emesso al trigger: si legge due
          // stati dopo, cioe' in S_CW2.
          S_CW0: state <= S_CW2;
          S_CW1: state <= S_CW2;   // non piu' raggiunto (rete di sicurezza)
          S_CW2: begin // harmony_q_b = COMMSTREAM pointer
            ptr_lat <= harmony_q_b;
            if (is_dsptr) begin
              harmony_addr_b <= ds_ptr_word(COMMSTREAM);
              harmony_wdata_b <= is_plus
                ? (({harmony_q_b[23:0], 8'd0} & 32'hff000000) | {8'd0, din, 16'd0})
                : (({harmony_q_b[23:0], 8'd0} & 32'hf0000000) | {4'd0, din, 20'd0});
              harmony_be_b <= 4'b1111; harmony_we_b <= 1'b1;
              state <= S_IDLE;
            end else begin
              if (disp_ok(harmony_q_b)) begin
                dba = disp_byte_addr(harmony_q_b);
                harmony_addr_b <= dba[14:2];
                harmony_wdata_b <= {4{din}};
                harmony_be_b <= 4'b0001 << dba[1:0];
                harmony_we_b <= 1'b1;
              end
              state <= S_CW3;
            end
          end
          S_CW3: begin // write back incremented COMMSTREAM pointer
            harmony_addr_b <= ds_ptr_word(COMMSTREAM);
            harmony_wdata_b <= ptr_lat + (is_plus ? 32'h00010000 : 32'h00100000);
            harmony_be_b <= 4'b1111; harmony_we_b <= 1'b1;
            state <= S_IDLE;
          end

          //------------------------------------------------------------
          // background: ff_val refresh (address issued in S_IDLE)
          S_FF0: state <= S_FF2;
          S_FF1: state <= S_FF2;   // non piu' raggiunto (rete di sicurezza)
          S_FF2: begin
            ff_val <= lane8(harmony_q_b, ff_offset[1:0]);
            state <= S_BG0;
          end
          S_FF3: state <= S_BG0; // unused, safety

          //------------------------------------------------------------
          // background: amplitude precompute.
          // waveform pointer reads pipelined at wf_base+0/4/8, then either
          // three display-RAM sample reads (3-voice) or one ROM/RAM sample
          // read (digital).
          // Catena a 6 letture in cascata (3 puntatori waveform + 3 campioni).
          // Ogni indirizzo emesso allo stato S si legge allo stato S+2, quindi
          // in ogni stato centrale si LEGGE un dato e si EMETTE l'indirizzo
          // successivo. Lo stato finale e' S_BG7 (prima era S_BG9).
          S_BG0: begin harmony_addr_b <= wf_base[14:2];          state <= S_BG1; end
          S_BG1: begin harmony_addr_b <= wf_base[14:2] + 13'd1;  state <= S_BG2; end
          S_BG2: begin
            wf_lat[0] <= harmony_q_b;                            // wf_base+0
            harmony_addr_b <= wf_base[14:2] + 13'd2;
            state <= S_BG3;
          end
          S_BG3: begin
            wf_lat[1] <= harmony_q_b;                            // wf_base+1
            if (digital_audio_en) begin
              dig_addr <= wf_lat[0] + (music_cnt[0] >> (is_plus ? 5'd13 : 5'd21));
              state <= S_BGD0;
            end else begin
              dba = wf_disp_byte(wf_lat[0], music_cnt[0], wave_size[0]);
              harmony_addr_b <= dba[14:2];                       // campione 0
              state <= S_BG4;
            end
          end
          S_BG4: begin
            wf_lat[2] <= harmony_q_b;                            // wf_base+2
            dba = wf_disp_byte(wf_lat[1], music_cnt[1], wave_size[1]);
            harmony_addr_b <= dba[14:2];                         // campione 1
            state <= S_BG5;
          end
          S_BG5: begin
            dba = wf_disp_byte(wf_lat[0], music_cnt[0], wave_size[0]);
            samp_lat[0] <= lane8(harmony_q_b, dba[1:0]);         // campione 0
            dba = wf_disp_byte(wf_lat[2], music_cnt[2], wave_size[2]);
            harmony_addr_b <= dba[14:2];                         // campione 2
            state <= S_BG6;
          end
          S_BG6: begin
            dba = wf_disp_byte(wf_lat[1], music_cnt[1], wave_size[1]);
            samp_lat[1] <= lane8(harmony_q_b, dba[1:0]);         // campione 1
            state <= S_BG7;
          end
          S_BG7: begin
            dba = wf_disp_byte(wf_lat[2], music_cnt[2], wave_size[2]);
            amp_cached <= samp_lat[0] + samp_lat[1] + lane8(harmony_q_b, dba[1:0]);
            state <= S_IDLE;
          end
          S_BG8: state <= S_IDLE;   // non piu' raggiunto (rete di sicurezza)
          S_BG9: state <= S_IDLE;   // non piu' raggiunto (rete di sicurezza)
          // digital sample
          S_BGD0: begin
            // Limite = taglia REALE della cartuccia (Stella: myImage.size()),
            // non ROM_DEPTH che e' la capacita' della M10K.
            if (dig_addr < {13'd0, rom_bytes}) begin
              rom_addr_b <= dig_addr[16:2];
              state <= S_BGD1;
            end else if (dig_addr[31:28] == 4'h4) begin
              // 2 ago 2026: era dig_addr[12:2], cioe' il campione digitale in RAM
              // veniva cercato solo nei primi 8 KB. Stella lo cerca fino a
              // 0x40008000 (CartCDF.cxx:290, "32K RAM" nel pannello del
              // debugger): con la RAM a 32 KB servono 13 bit di word.
              // E' il percorso dell'audio rovinato di Tutankham Arcade.
              harmony_addr_b <= dig_addr[14:2];
              state <= S_BGD1;
            end else begin
              amp_cached <= 8'd0;
              state <= S_IDLE;
            end
          end
          S_BGD1: state <= S_BGD3;   // indirizzo emesso in S_BGD0 -> letto in S_BGD3
          S_BGD2: state <= S_BGD3;   // non piu' raggiunto (rete di sicurezza)
          S_BGD3: begin
            // stesso limite usato in S_BGD0 per scegliere il ramo: se qui si
            // usasse ROM_DEPTH (capacita' della M10K) invece di rom_bytes
            // (taglia reale della cartuccia) i due stati potrebbero scegliere
            // memorie diverse per lo stesso indirizzo.
            sv = (dig_addr < {13'd0, rom_bytes}) ? lane8(rom_q_b, dig_addr[1:0])
                                                 : lane8(harmony_q_b, dig_addr[1:0]);
            if ((music_cnt[0] & (32'd1 << (is_plus ? 5'd12 : 5'd20))) == 32'd0) sv = sv >> 4;
            amp_cached <= sv & 8'h0F;
            state <= S_IDLE;
          end

          default: state <= S_IDLE;
        endcase
      end
    end
  end

  //--------------------------------------------------------------------------
  // ARM bus response (2-cycle handshake) + minimal LPC peripheral stub
  //--------------------------------------------------------------------------
  // Livello 1: questo adapter gira ORA su clk_vid (dominio ARM). Serve
  // bus_req/bus_ack/bus_rdata DIRETTAMENTE dalle porte A M10K (rom_q_a /
  // harmony_q_a, entrambe registrate su clk_vid) e dai registri periferici LPC,
  // senza alcun attraversamento CDC. La sequenza arm_p1 / bus_ack e' identica a
  // prima (stessa golden trace): l'unica differenza e' il dominio di clock.
  // I registri periferici LPC (t1tcr/systick/mamcr) vivono ora in clk_vid:
  // nessun altro li usa su clk_sys, quindi lo spostamento e' sicuro.
  // The M10K ports have 2-cycle read latency (registered address AND output):
  // e0 addr -> e1 addr captured (arm_p1) -> e2 q valid + ack; bus_rdata is a
  // combinational mux of the registered RAM outputs, sampled by the core at e3.
  reg arm_p1; reg [3:0] arm_region_d;
  reg  bus_ack_r;
  wire bus_first = bus_req && !arm_p1 && !bus_ack_r;
  // STADIO 3b - ACK A 1 CICLO PER LE LETTURE CON INDIRIZZO ANTICIPATO.
  //
  // Lo stadio 2 presenta l'indirizzo alla porta A della RAM Harmony gia'
  // durante S_EXEC: per quegli accessi `harmony_q_a` e' valido nel PRIMO ciclo
  // di S_MEM e l'ack puo' salire subito.
  //
  // NON vale per il fetch ne' per le letture da ROM: `rom_addr_a` segue
  // `bus_addr` (non anticipato) perche' alimenta anche `bus_rdata_next`, la
  // parola che va nella cache istruzioni. Dare l'ack a 1 ciclo anche a loro e'
  // esattamente il difetto che ha fatto fallire i primi quattro tentativi: la
  // prima istruzione fetchata tornava spazzatura (`f040` invece di `480f`).
  //
  // `pre_hit` e' vero solo se l'indirizzo presentato il ciclo scorso e' proprio
  // quello richiesto adesso, e sta nella regione 4'h4.
  reg [31:0] pre_addr_d;
  reg        pre_armed_d;
  always @(posedge clk_vid) begin
    if (rst_vid) begin pre_armed_d <= 1'b0; pre_addr_d <= 32'd0; end
    else         begin pre_armed_d <= bus_req_pre; pre_addr_d <= bus_addr_pre; end
  end
  wire pre_hit = bus_req && pre_armed_d && !bus_we &&
                 (bus_addr[31:28] == 4'h4) &&
                 (pre_addr_d == {bus_addr[31:2], 2'b00});
  assign bus_ack = (bus_we || pre_hit) ? bus_first : bus_ack_r;
  reg [31:0] t1tcr, t1tc, systick_ctrl, systick_reload, systick_count, mamcr;
  reg [31:0] periph_rdata;
  always @(posedge clk_vid) begin
    if (rst_vid) begin
      arm_p1 <= 0; bus_ack_r <= 0; arm_region_d <= 0;
      t1tcr <= 0; t1tc <= 0; systick_ctrl <= 32'h4; systick_reload <= 0;
      systick_count <= 0; mamcr <= 0; periph_rdata <= 0;
    end else begin
      if (t1tcr[0]) t1tc <= t1tc + 1'd1;
      if (systick_ctrl[0]) begin
        if (systick_count == 0) systick_count <= systick_reload;
        else systick_count <= systick_count - 1'd1;
      end

      // Step 4 speed: le WRITE committano gia' nel primo ciclo req-visibile
      // (harmony_we_a e' combinatorio, e il blocco di cattura qui sotto
      // scrive i registri periferici nello stesso ciclo) — l'ack di una
      // write puo' quindi salire subito, senza aspettare la latenza di
      // lettura M10K. Le READ mantengono il percorso a 2 stadi (arm_p1).
      // 4 agosto 2026: con `outdata_reg_a = "UNREGISTERED"` sulle due M10K la
      // porta A ha UN ciclo di latenza invece di due. L'indirizzo viene
      // catturato nel ciclo di accettazione e il dato e' gia' valido nello
      // stesso ciclo in cui prima si alzava soltanto `arm_p1`: quindi l'ack
      // puo' salire subito e lo stadio `arm_p1` non serve piu' nemmeno per le
      // letture. Una lettura ARM passa da 3 cicli di S_MEM a 2.
      // `arm_p1` resta come segnale (lo legge la logica dei registri qui sotto)
      // ma non e' piu' sul percorso dell'ack.
      // `!pre_hit`: l'accesso veloce ha gia' avuto l'ack combinatorio,
      // un secondo ack il ciclo dopo farebbe avanzare due volta la FSM.
      bus_ack_r <= bus_first && !bus_we && !pre_hit;
      arm_p1 <= 1'b0;
      if (bus_first) begin
        arm_region_d <= bus_addr[31:28];
        // if/else-if instead of `case (bus_addr)`: a 32-bit case expression
        // makes Quartus unable to verify completeness (warning 10762) even
        // with a default arm; explicit compares avoid that and match the
        // same six known peripheral addresses one-for-one.
        if (bus_we) begin
          if      (bus_addr == 32'hE0008004) t1tcr <= bus_wdata;
          else if (bus_addr == 32'hE0008008) t1tc <= bus_wdata;
          else if (bus_addr == 32'hE000E010) systick_ctrl <= {systick_ctrl[31:17], bus_wdata[16], systick_ctrl[15:3], bus_wdata[2:0]};
          else if (bus_addr == 32'hE000E014) systick_reload <= bus_wdata & 32'h00FFFFFF;
          else if (bus_addr == 32'hE000E018) systick_count <= bus_wdata & 32'h00FFFFFF;
          else if (bus_addr == 32'hE01FC000) mamcr <= bus_wdata;
        end else begin
          if      (bus_addr == 32'hE0008004) periph_rdata <= t1tcr;
          else if (bus_addr == 32'hE0008008) periph_rdata <= t1tc;
          else if (bus_addr == 32'hE000E010) periph_rdata <= systick_ctrl;
          else if (bus_addr == 32'hE000E014) periph_rdata <= systick_reload;
          else if (bus_addr == 32'hE000E018) periph_rdata <= systick_count;
          else if (bus_addr == 32'hE01FC000) periph_rdata <= mamcr;
          else                                periph_rdata <= 32'd0;
        end
      end
    end
  end
  // Sul percorso veloce il dato viene consumato NELLO STESSO ciclo di
  // `bus_first`, quando `arm_region_d` porta ancora la regione dell'accesso
  // precedente: la selezione va forzata a 4'h4, che e' l'unica regione per cui
  // `pre_hit` puo' essere vero.
  wire [3:0] region_now = pre_hit ? 4'h4 : arm_region_d;
  assign bus_rdata = (region_now == 4'h0) ? rom_q_a :
                     (region_now == 4'h4) ? harmony_q_a : periph_rdata;
  // FETCH a 64 bit: la ROM sdoppiata restituisce anche la word SUCCESSIVA senza
  // costare un secondo accesso. Vale solo per la regione 0 (ROM), l'unica da cui
  // l'ARM esegue codice: `bus_rdata_next_ok` dice al core quando puo' fidarsi.
  assign bus_rdata_next    = rom_q_a_next;
  assign bus_rdata_next_ok = (arm_region_d == 4'h0);

endmodule
`default_nettype wire
