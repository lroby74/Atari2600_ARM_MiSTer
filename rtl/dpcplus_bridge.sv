// DPC+ cartridge bridge - PASSO 1 ESTESO.
//   implementato: fetcher, Fast Fetch redirect, CALLFUNCTION func 0/1/2
//   stub (PASSO 2): RNG, motore musicale/AMPLITUDE, CALLFUNCTION 254/255 (ARM)
//
// Semantica da Stella CartDPCPlus.cxx (stella-master/src/emucore/). Modulo
// INDIPENDENTE dal motore CDF: nessuno stato condiviso con cdf_bridge.sv.
//
// MAPPA MEMORIA (CartDPCPlus.cxx:36-47)
//   ROM 32KB: myProgramImage = image + 3KB, size 32768-3072 = 29696
//             (6 banchi da 4KB a 3072..27647 + 5KB di immagine iniziale)
//   RAM 8KB:  myDisplayImage   = RAM + 3KB, size 4096
//             myFrequencyImage = RAM + 7KB, size 1024
//   setInitialState (riga 106-109): RAM azzerata, poi 5KB copiati da
//   ProgramImage+24KB -> DisplayImage (sconfina di 1KB nella tabella
//   frequenze: e' cosi' che Stella inizializza le frequenze).
//
// PEEK (CartDPCPlus.cxx:255-437), ordine ESATTO:
//   1) peekvalue = ProgramImage[bank*4KB + off]      <- sempre letto
//   2) se FastFetch && LDAimmediate && peekvalue<0x28 -> addr = peekvalue
//   3) LDAimmediate = 0 (sempre)
//   4) se addr < 0x28 -> lettura registro
//      altrimenti     -> hotspot bank, LDAimmediate = FastFetch && (peek==0xA9),
//                        ritorna peekvalue
//   NB: il byte "visto" e' quello che il bridge stesso restituisce - la
//   cartuccia serve TUTTO $1000-$1FFF, quindi non serve alcun observer esterno.
`default_nettype none
module dpcplus_bridge #(
  parameter ROM_FILE = "", parameter RAM_FILE = ""
)(
  input  wire        clk,            // clk_sys: TUTTA la logica DPC+ vive qui
  // STEP A: clock della porta ARM delle memorie. Serve SOLO a pilotare
  // clock0 delle due M10K dual-clock; nessuna logica di questo modulo e'
  // sensibile a clk_vid in questo step (l'accesso ARM arriva in STEP B/C).
  input  wire        clk_vid,
  input  wire        rst,
  input  wire [15:0] m6502_addr,
  input  wire [7:0]  m6502_din,
  input  wire        m6502_rwn,
  output reg  [7:0]  m6502_dout,
  output wire [14:0] rom_a,
  output reg         callfn,         // CALLFUNCTION 0xFE/0xFF -> ARM (PASSO 2)
  output reg  [7:0]  callfn_val,
  input  wire        rom_load_we,
  input  wire [14:0] rom_load_addr,
  input  wire [7:0]  rom_load_data,
  // STEP B: bus ARM (clk_vid) verso le porte A delle memorie DPC+.
  // Stesso protocollo del bus di cdf_bridge: req/ack, latenza lettura M10K 2
  // cicli assorbita da arm_p1. Mappa regioni ARM del driver DPC+:
  //   region 0 (0x0.......) -> ROM  (fetch/literal)
  //   region 4 (0x4.......) -> RAM  (DPCRAM 8KB: driver + display + frequenze)
  // In STEP B il thumb_core NON e' ancora abilitato per DPC+ (il suo reset
  // resta legato a is_cdf_mapper=BANKCDF, STEP C): qui si costruisce solo il
  // percorso, che resta quindi inattivo (bus_req_arm = 0).
  input  wire        rst_vid,
  input  wire [31:0] bus_addr,
  input  wire [31:0] bus_wdata,
  output wire [31:0] bus_rdata,
  input  wire [3:0]  bus_be,
  input  wire        bus_we,
  input  wire        bus_req,
  output wire        bus_ack,
  // Indirizzo anticipato (stadio 2): il core annuncia l'accesso gia'
  // durante S_EXEC, cosi' la M10K lo cattura un ciclo prima.
  input  wire [31:0] bus_addr_pre,
  input  wire        bus_req_pre,
  output wire [2:0]  dbg_bank,
  output wire [7:0]  dbg_ff
);

  localparam [15:0] PROG_SIZE = 16'd29696;   // myProgramImage.size()
  localparam [12:0] DISP_SIZE = 13'd4096;    // myDisplayImage.size()
  localparam [14:0] PROG_BASE = 15'd3072;    // ROM + 3KB
  localparam [12:0] DISP_BASE = 13'd3072;    // RAM + 3KB

  //--------------------------------------------------------------------------
  // Memorie dedicate (M10K). Latenza lettura 2 clk.
  //--------------------------------------------------------------------------
  // Accesso ARM a 32 bit sulle memorie DPC+ (2 agosto 2026: ora NATIVO).
  //
  // thumb_core presenta SEMPRE bus_addr allineato alla word (& 32'hFFFFFFFC,
  // vedi thumb_core.sv OP_LDR*/OP_STR*) e si aspetta indietro una word intera,
  // usando bus_be per selezionare le lane in scrittura. Le memorie DPC+ hanno
  // ora la porta A a 32 bit come quelle del cdf_bridge, quindi si serve
  // direttamente: niente contatore di lane, niente composizione.
  wire [31:0] arm_rom_q, arm_ram_q;
  // 6 ago 2026: tolto `arm_is_rom`, assegnato e mai letto (warning 10036).
  // La regione ROM si riconosce gia' per esclusione da `arm_is_ram`.
  wire       arm_is_ram = (bus_addr[31:28] == 4'h4);
  // STADIO 2 - INDIRIZZO ANTICIPATO. Quando il core annuncia l'accesso
  // si presenta gia' il suo indirizzo alla porta A della RAM.
  wire [31:0] arm_addr_eff = bus_req_pre ? bus_addr_pre : bus_addr;
  // FIX GLOBALE punto 4: regione 0xE = periferiche LPC (timer 1, SysTick,
  // MAMCR). Prima era indecodificata: una lettura ricadeva sul ramo RAM e
  // restituiva un byte qualunque della DPCRAM, una scrittura veniva scartata.
  // cdf_bridge ha lo stesso stub dal Livello 1 (cdf_bridge.sv:22-23, 686-736).
  wire       arm_is_periph = (bus_addr[31:28] == 4'hE);

  // ACCESSO ARM DIRETTO (2 agosto 2026). Sostituisce il cammino a 4 lane.
  //
  // Prima: 4 fasi per ogni lane con contatore `arm_ph`, perche' le M10K erano a
  // 8 bit su ENTRAMBE le porte. Costo: 6 cicli clk_vid per lettura, 4 per
  // scrittura, contro i 2 e 1 del cdf_bridge, che le sue memorie le ha a 32 bit
  // (cdf_m10k_memories.sv:33, harmony_m10k_tdp.sv:59). L'ottimizzazione
  // Livello 1 era stata data al solo motore CDF: questo e' il recupero.
  //
  // MISURATO il 1-2 ago su TUTTE E SEI le ROM DPC+ rotte del banco (24 frame,
  // sonda tb_full_system.sv:435 = scritture ARM in Display Data con
  // tia_vblank=0, cioe' mentre la TIA disegna gia' la parte visibile):
  //   DK Arcade      6365/11293 = 56%     Epic Adventure  3710/16590 = 22%
  //   Rightris       9340/29204 = 32%     Meooow!         4110/19418 = 21%
  //   Lucky Chase    5462/19634 = 28%     Meooow! 2       3632/19088 = 19%
  // Nessuna eccezione, e l'ordine coincide con la gravita' del sintomo su
  // hardware (DK Arcade e' il piu' rotto ed e' il primo della lista).
  //
  // Ora: le porte A sono a 32 bit e l'indirizzo di word esce COMBINATORIO da
  // bus_addr, che thumb_core tiene stabile per tutta la transazione. Handshake
  // identico a cdf_bridge.sv:721-722: lettura 2 cicli, scrittura 1.
  reg  [14:0] rom_addr_r;
  wire [7:0]  rom_q;
  dpcp_rom_m10k #(.INIT_FILE(ROM_FILE)) rom_i (
    .clk_a(clk_vid), .addr_a(bus_addr[14:2]), .q_a(arm_rom_q),
    .clk_b(clk),     .addr_b(rom_load_we ? rom_load_addr : rom_addr_r),
    .data_b(rom_load_data), .wren_b(rom_load_we), .q_b(rom_q)
  );

  // SCRITTURA ARM: una sola word, un solo ciclo. Le guardie che restano:
  //   bus_req & bus_we   -> e' davvero uno store
  //   arm_is_ram         -> la ROM non e' scrivibile
  //   ~bus_ack           -> scrive una volta sola (bus_ack sale al ciclo dopo)
  // I byte enable arrivano da bus_be e bus_wdata e' gia' replicato da
  // thumb_core per STRB/STRH ({4{r[..][7:0]}} / {2{r[..][15:0]}}), quindi la
  // lane giusta prende il byte giusto senza altra logica.
  // `~bus_ack_r` e non `~bus_ack`: pre_hit e' alto solo sulle LETTURE,
  // quindi per le scritture i due coincidono, ma il riferimento al
  // registro tiene fuori il percorso combinatorio nuovo.
  wire arm_wr_word = bus_req & bus_we & arm_is_ram & ~bus_ack_r;

  //--------------------------------------------------------------------------
  // FIX GLOBALE punto 3 - INIZIALIZZAZIONE DPCRAM.
  //
  // Stella CartridgeDPCPlus::setInitialState() (CartDPCPlus.cxx:104-109):
  //     myDPCRAM.fill(0);
  //     copy_n(myProgramImage.data() + 24_KB, 5_KB, myDisplayImage.begin());
  // con myProgramImage = image + 3KB  -> sorgente  = image[27648 .. 32767]
  //     myDisplayImage = DPCRAM + 3KB -> destinaz. = DPCRAM[3072 .. 8191]
  // (i 5 KB sconfinano di 1 KB oltre la Display Data dentro la tabella
  //  frequenze: e' voluto, e' cosi' che Stella inizializza le frequenze).
  //
  // Schema IDENTICO a quello che cdf_bridge usa per la Harmony RAM
  // (cdf_bridge.sv:125-131 e 160-164): si sfrutta lo stream del loader invece
  // di una FSM di copia dedicata, quindi costo zero in tempo e nessuna
  // contesa sulla porta B con il motore 6502 (che durante il download e'
  // comunque tenuto in reset). Due finestre DISGIUNTE sullo stesso stream:
  //     rom_load_addr <  8192  -> scrive 0           (myDPCRAM.fill(0))
  //     rom_load_addr >= 27648 -> scrive il byte ROM (copia dei 5 KB)
  // La seconda finestra arriva DOPO la prima nello stream: l'ordine e'
  // esattamente quello di Stella (prima l'azzeramento, poi la copia).
  //
  // ARITMETICA: 24576 = 3*8192, quindi per rom_load_addr nella finestra
  // 27648..32767 vale rom_load_addr[12:0] == rom_load_addr - 24576, cioe'
  // 3072..8191 - la destinazione esatta. Nella finestra 0..8191 vale
  // rom_load_addr[12:0] == rom_load_addr. Un solo troncamento serve entrambi
  // i casi, nessun sottrattore.
  //
  // SCELTA DICHIARATA (differenza voluta da Stella): Stella richiama
  // setInitialState() anche da reset(); qui si inizializza SOLO al download.
  // Sull'hardware Harmony reale un reset di console NON ricarica la RAM ARM -
  // l'immagine viene scritta una volta sola all'inserimento della cartuccia.
  // Il download-time init e' quindi piu' vicino all'hardware che al
  // riferimento emulatore.
  //--------------------------------------------------------------------------
  localparam [14:0] DPCRAM_ZERO_BYTES = 15'd8192;    // myDPCRAM.fill(0)
  localparam [14:0] DISP_SRC_BASE     = 15'd27648;   // ProgramImage + 24 KB
  wire        ram_init_copy = rom_load_we && (rom_load_addr >= DISP_SRC_BASE);
  wire        ram_init_zero = rom_load_we && (rom_load_addr <  DPCRAM_ZERO_BYTES);
  wire        ram_init_we   = ram_init_zero | ram_init_copy;
  wire [12:0] ram_init_addr = rom_load_addr[12:0];
  wire [7:0]  ram_init_data = ram_init_copy ? rom_load_data : 8'd0;

  //--------------------------------------------------------------------------
  // RICONOSCIMENTO DEL DRIVER DPC+ -> maschera DFxFRACLOW (1 agosto 2026).
  //
  // Stella, CartDPCPlus.cxx:64-78 e CartDPCPlus.hxx:305-313:
  //   "Older DPC+ driver code had different behaviour wrt the mask used to
  //    retrieve 'DFxFRACLOW'. ROMs built with an old DPC+ driver and using the
  //    newer mask can result in 'jittering' in the playfield display.
  //    For current versions this is 0x0F00FF; older versions need 0x0F0000."
  // Il DEFAULT di Stella e' 0x0F00FF ("Jitter"); solo due MD5 di driver su
  // quattro noti passano a 0x0F0000 ("Stable"). Questo RTL implementava la
  // sola variante Stable, cioe' la MINORANZA: su un driver Jitter la scrittura
  // di DFxFRACLOW azzerava la parte frazionaria che invece va CONSERVATA, e
  // Stella dice esplicitamente che la differenza si vede NEL PLAYFIELD.
  //
  // MISURATO sulle 14 ROM DPC+ del banco (MD5 dei primi 3 KB):
  //   5f80b5a5... Stable  : dk, mw2, de, cg_no, cg_yes, mw, rt, sf   (8)
  //   17884ec1... Jitter  : lc, st, sv, te                           (4)
  //   b328dbdf... Jitter  : fr                                       (1)
  //   20d76d13... ignoto  : ea  -> per Stella e' il DEFAULT = Jitter (1)
  // Quindi 6 ROM su 14 erano trattate con la maschera sbagliata.
  //
  // COME: l'MD5 e' fuori questione in RTL. Si usa un CRC-32 (stesso polinomio
  // di zlib, init/final FFFFFFFF) calcolato sui primi 3072 byte MENTRE
  // scorrono nello stream del loader - costo zero in tempo, come per l'init
  // della DPCRAM. Il confronto e' con la costante del driver Stable presente
  // nel campione.
  //
  // LIMITE DICHIARATO: dei due driver Stable di Stella qui e' verificabile
  // solo `5f80b5a5adbe483addc3f6e6f1b472f8` (CRC 0xA45A045F); dell'altro
  // (`8dd73b44fd11c488326ce507cbeb19d1`, "Stable NOT Encore Compatible") non
  // esiste alcuna immagine nel repo, quindi il suo CRC non e' calcolabile e
  // una cartuccia che lo usa ricadrebbe sul default Jitter. E' comunque il
  // comportamento che Stella riserva a un driver non riconosciuto. Lo slot per
  // la seconda costante e' pronto qui sotto: basta aggiungere l'OR.
  //
  // Il CRC vive FUORI dal blocco resettato da `rst`: al momento del download
  // `mapper` non e' ancora deciso, quindi `dpcp_active` (e con esso `rst`) puo'
  // valere 0 - la stessa ragione per cui l'init della DPCRAM e' pilotato
  // direttamente dallo stream e non dalla FSM.
  //--------------------------------------------------------------------------
  localparam [14:0] DRIVER_BYTES = 15'd3072;
  // Stella riconosce DUE driver "Stable" per MD5 dei primi 3 KB
  // (CartDPCPlus.cxx:64-78); tutti gli altri prendono il default JITTER:
  //   5f80b5a5adbe483addc3f6e6f1b472f8  Stable  Encore Compatible
  //   8dd73b44fd11c488326ce507cbeb19d1  Stable  NOT Encore Compatible
  // Noi usiamo il CRC-32 degli stessi 3072 byte al posto dell'MD5.
  // 3 agosto 2026: il secondo era rimasto un segnaposto vuoto, quindi i titoli
  // col driver 8dd73b44 cadevano su JITTER. Con JITTER il byte basso di
  // fcnt viene CONSERVATO invece che azzerato dalla scrittura a DFxFRACLOW:
  // il contatore frazionario accumula e il playfield striscia verso l'alto a
  // velocita' costante riavvolgendo. E' il sintomo di Evil Magician Returns II
  // (il pannello di Stella mostra proprio "Ver = 8dd73b44fd11c488326ce507cbeb19d1").
  // CRC-32 misurati sui file reali: 5f80b5a5 -> A45A045F (18 ROM, gia' giusto),
  // 8dd73b44 -> 5F942217 (EpicAdventureV22.bin e Evil Magician Returns demo.bin).
  localparam [31:0] CRC_DRIVER_STABLE_A = 32'hA45A045F;  // 5f80b5a5... (Encore)
  localparam [31:0] CRC_DRIVER_STABLE_B = 32'h5F942217;  // 8dd73b44... (non-Encore)

  function [31:0] crc32_byte;
    input [31:0] c; input [7:0] d;
    integer n; reg [31:0] x;
    begin
      x = c ^ {24'd0, d};
      for (n = 0; n < 8; n = n + 1)
        x = x[0] ? ((x >> 1) ^ 32'hEDB88320) : (x >> 1);
      crc32_byte = x;
    end
  endfunction

  reg [31:0] drv_crc;
  always @(posedge clk) begin
    if (rom_load_we) begin
      if (rom_load_addr == 15'd0)          drv_crc <= crc32_byte(32'hFFFFFFFF, rom_load_data);
      else if (rom_load_addr < DRIVER_BYTES) drv_crc <= crc32_byte(drv_crc, rom_load_data);
    end
  end
  wire frac_stable = ((~drv_crc) == CRC_DRIVER_STABLE_A) ||
                     ((~drv_crc) == CRC_DRIVER_STABLE_B);

  reg  [12:0] ram_addr_a; reg [7:0] ram_wdata_a; reg ram_we_a;
  wire [7:0]  ram_q_a;
  dpcp_ram_m10k #(.INIT_FILE(RAM_FILE)) ram_i (
    .clk_a(clk_vid), .addr_a(arm_addr_eff[12:2]), .data_a(bus_wdata),
    .be_a(bus_be), .wren_a(arm_wr_word), .q_a(arm_ram_q),
    .clk_b(clk),
    .addr_b (ram_init_we ? ram_init_addr : ram_addr_a),
    .data_b (ram_init_we ? ram_init_data : ram_wdata_a),
    .wren_b (ram_init_we | ram_we_a),
    .q_b    (ram_q_a)
  );

  // Handshake ARM (2 agosto 2026): lettura 2 cicli, scrittura 1. Identico a
  // cdf_bridge.sv:704-746, ora che anche qui le porte A sono a 32 bit.
  //   e0  indirizzo presentato (combinatorio da bus_addr) -> arm_p1
  //   e1  indirizzo catturato dalla M10K                  -> bus_ack
  //   e2  q_a valido e ack visibile: il core campiona bus_rdata
  // Le SCRITTURE non aspettano la latenza di lettura: arm_wr_word e' gia' alto
  // nel primo ciclo, quindi l'ack puo' salire subito.
  //
  // La regione va RICORDATA (arm_region_d) perche' bus_rdata e' ora una mux
  // combinatoria delle uscite registrate, non piu' una word composta a mano:
  // quando l'ack arriva, bus_addr potrebbe gia' essere cambiato.
  //
  // FIX GLOBALE punto 4: la regione 0xE = periferiche LPC (timer 1, SysTick,
  // MAMCR). Registri e indirizzi replicano uno-a-uno lo stub di cdf_bridge.
  reg        arm_p1;
  reg [3:0]  arm_region_d;
  // STADIO 3b - ACK A 1 CICLO PER LE LETTURE CON INDIRIZZO ANTICIPATO.
  // Vale SOLO per la regione 4'h4 (RAM ARM): e' l'unica porta che riceve
  // l'indirizzo in anticipo. La ROM resta a 2 cicli - darle l'ack anticipato
  // significa consegnare al fetch una word non ancora letta.
  reg        bus_ack_r;
  reg [31:0] pre_addr_d;
  reg        pre_armed_d;
  always @(posedge clk_vid) begin
    if (rst_vid) begin pre_armed_d <= 1'b0; pre_addr_d <= 32'd0; end
    else         begin pre_armed_d <= bus_req_pre; pre_addr_d <= bus_addr_pre; end
  end
  wire pre_hit = bus_req && pre_armed_d && !bus_we &&
                 (bus_addr[31:28] == 4'h4) &&
                 (pre_addr_d == {bus_addr[31:2], 2'b00});
  wire bus_first = bus_req && !arm_p1 && !bus_ack_r;
  assign bus_ack = pre_hit ? bus_first : bus_ack_r;
  reg [31:0] t1tcr, t1tc, systick_ctrl, systick_reload, systick_count, mamcr;
  reg [31:0] periph_rdata;
  always @(posedge clk_vid) begin
    if (rst_vid) begin
      bus_ack_r <= 1'b0; arm_p1 <= 1'b0; arm_region_d <= 4'd0;
      periph_rdata <= 32'd0;
      t1tcr <= 32'd0; t1tc <= 32'd0; mamcr <= 32'd0;
      systick_ctrl <= 32'h4; systick_reload <= 32'd0; systick_count <= 32'd0;
    end else begin
      // contatori liberi (identici a cdf_bridge.sv:710-714). Sono self-gated:
      // t1tcr/systick_ctrl possono essere scritti SOLO da un accesso ARM, che
      // richiede bus_req - a 0 per costruzione con qualunque mapper != DPC+.
      if (t1tcr[0]) t1tc <= t1tc + 1'd1;
      if (systick_ctrl[0]) begin
        if (systick_count == 32'd0) systick_count <= systick_reload;
        else                        systick_count <= systick_count - 1'd1;
      end

      // 4 ago 2026: con le porte A non registrate il dato e' gia' valido nel
      // ciclo in cui prima si alzava soltanto arm_p1, quindi l'ack sale subito
      // anche per le letture e lo stadio arm_p1 esce dal percorso.
      // Lettura ARM: 3 cicli di S_MEM -> 2. Stessa modifica di cdf_bridge.sv.
      // `!pre_hit`: l'accesso veloce ha gia' avuto l'ack combinatorio.
      bus_ack_r <= bus_first && !pre_hit;
      arm_p1  <= 1'b0;
      if (bus_first) begin
        arm_region_d <= bus_addr[31:28];
        if (arm_is_periph) begin
          // confronti espliciti invece di case(bus_addr): un case su
          // espressione a 32 bit fa scattare il warning Quartus 10762
          // (completezza non verificabile). Stessa scelta di cdf_bridge.
          if (bus_we) begin
            if      (bus_addr == 32'hE0008004) t1tcr <= bus_wdata;
            else if (bus_addr == 32'hE0008008) t1tc  <= bus_wdata;
            else if (bus_addr == 32'hE000E010) systick_ctrl <= {systick_ctrl[31:17], bus_wdata[16], systick_ctrl[15:3], bus_wdata[2:0]};
            else if (bus_addr == 32'hE000E014) systick_reload <= bus_wdata & 32'h00FFFFFF;
            else if (bus_addr == 32'hE000E018) systick_count  <= bus_wdata & 32'h00FFFFFF;
            else if (bus_addr == 32'hE01FC000) mamcr <= bus_wdata;
          end else begin
            if      (bus_addr == 32'hE0008004) periph_rdata <= t1tcr;
            else if (bus_addr == 32'hE0008008) periph_rdata <= t1tc;
            else if (bus_addr == 32'hE000E010) periph_rdata <= systick_ctrl;
            else if (bus_addr == 32'hE000E014) periph_rdata <= systick_reload;
            else if (bus_addr == 32'hE000E018) periph_rdata <= systick_count;
            else if (bus_addr == 32'hE01FC000) periph_rdata <= mamcr;
            else                               periph_rdata <= 32'd0;
          end
        end
      end
    end
  end
  // Sul percorso veloce il dato viene consumato NELLO STESSO ciclo di
  // `bus_first`, quando `arm_region_d` porta ancora la regione precedente.
  wire [3:0] region_now = pre_hit ? 4'h4 : arm_region_d;
  assign bus_rdata = (region_now == 4'h0) ? arm_rom_q :
                     (region_now == 4'h4) ? arm_ram_q : periph_rdata;

  //--------------------------------------------------------------------------
  // Stato fetcher + controllo
  //--------------------------------------------------------------------------
  reg [7:0]  tops [0:7];
  reg [7:0]  bots [0:7];
  reg [11:0] cnt  [0:7];
  reg [19:0] fcnt [0:7];
  reg [7:0]  finc [0:7];
  reg [2:0]  bank_reg;
  reg        fast_fetch;
  reg        lda_imm;                 // myLDAimmediate
  reg [3:0]  param_ptr;               // 4 bit: Stella limita a <8
  reg [7:0]  param [0:7];

  assign dbg_bank = bank_reg;
  // 6 ago 2026: `pend_idx` sui tre bit alti, finora a zero. Serve solo a far
  // vedere a Quartus che il segnale e' letto (il suo vero lettore e' il
  // testbench, che il tool non analizza): chiude il warning 10036 senza
  // togliere la sonda. I due bit bassi non cambiano, quindi chi legge gia'
  // `dbg_ff` non se ne accorge.
  assign dbg_ff   = {3'd0, pend_idx, lda_imm, fast_fetch};
  assign rom_a    = rom_addr_r;

  wire        cs      = (m6502_addr[15:12] == 4'h1);
  wire [11:0] off     = m6502_addr[11:0];
  reg  [15:0] addr_d;
  reg         rwn_d;
  //--------------------------------------------------------------------------
  // BUG CORRETTO (1 agosto 2026) - SCRITTURE INDICIZZATE PERSE.
  //
  // `trigger` era solo `cs && (m6502_addr != addr_d)`. Ma il 6502, su
  //     STA abs,X   STA abs,Y   STA (zp),Y
  // senza attraversamento di pagina, emette DUE cicli di bus con lo STESSO
  // indirizzo: un ciclo di dummy-READ (l'indirizzo non ancora corretto nel
  // byte alto) e subito dopo il ciclo di SCRITTURA vero. L'indirizzo non
  // cambia fra i due, quindi il secondo NON generava trigger e la scrittura
  // veniva SCARTATA IN SILENZIO.
  //
  // MISURATO su DK Arcade (120k cicli CPU, sonda [perso] nel testbench):
  // 104 scritture perse, TUTTE nella finestra registri, cosi' distribuite:
  //     fn=0 DFxFRACLOW 48 | fn=1 DFxFRACHI 48 | fn=5 DFxLOW 4 | fn=8 DFxHI 4
  // Fra queste: `DF4FRACLOW <= 08` e `DF4FRACHI <= 0A` (puntatore 0x0A08) e
  // `DF6FRACLOW <= A5` / `DF6FRACHI <= 0B` (0x0BA5). Persi quei quattro byte,
  // fcnt[4] e fcnt[6] restano a ZERO e DF4FRACDATA/DF6FRACDATA leggono per
  // sempre disp[0] = 0. Il testbench (+col) mostra che quei due stream
  // alimentano esattamente COLUPF e COLUBK: playfield e sfondo NERI.
  // 0x0A08 e' anche la destinazione delle copie CALLFUNCTION 1 osservate
  // ([copia] dst=a08): il gioco ci scrive i colori e poi non li rilegge mai.
  //
  // Il discriminante e' il VERSO del ciclo, non il valore del dato: un
  // dummy-read seguito da una scrittura fa passare rwn da 1 a 0. Aggiungere
  // quel confronto costa un flip-flop e NON puo' scattare durante uno stallo
  // RDY (WSYNC), dove indirizzo E verso restano entrambi fermi - motivo per
  // cui NON si usa un semplice strobe di ciclo CPU (phi1): quello
  // ri-eseguirebbe la lettura ad ogni ciclo di stallo, e se il 6507 si ferma
  // su un operando ridiretto da Fast Fetch il contatore del fetcher
  // avanzerebbe a vuoto.
  //
  // LIMITE DICHIARATO: le read-modify-write (INC/DEC/ASL/ROL/LSR/ROR su un
  // registro) emettono READ, WRITE(vecchio), WRITE(nuovo) sullo stesso
  // indirizzo; qui passa solo la prima delle due scritture. I registri DPC+
  // sono write-only (una lettura in $028-$07F torna il byte di programma),
  // quindi nessuna cartuccia li tratta come locazioni RMW.
  //--------------------------------------------------------------------------
`ifdef DPCP_OLD_TRIGGER
  // solo per bisezione/confronto in simulazione: comportamento PRE-fix
  wire        trigger = cs && (m6502_addr != addr_d);
`else
  wire        trigger = cs && ((m6502_addr != addr_d) || (m6502_rwn != rwn_d));
`endif

  wire [2:0] w_index = off[2:0];
  wire [3:0] w_func  = off[6:3] - 4'd5;        // ((addr-0x28)>>3)&0xF
  wire       is_reg_w = (off >= 12'h028) && (off < 12'h080);
  wire       is_hot   = (off >= 12'hFF6) && (off <= 12'hFFB);

  reg [11:0] off_l;                    // off latchato al trigger
  wire is_hot_l = (off_l >= 12'hFF6) && (off_l <= 12'hFFB);

  // count = min(param[3], PROG_SIZE-ROMdata, DISP_SIZE-destBase), 0 se fuori
  // range (CartDPCPlus.cxx:194-201).
  function [8:0] cp_limit;
    input [15:0] src; input [11:0] dst; input [7:0] n;
    reg [15:0] lr; reg [12:0] ld; reg [8:0] r;
    begin
      // Assegnazioni INCONDIZIONATE (1 ago 2026, warning Quartus 10776).
      // In Verilog le function sono STATIC: le variabili locali sono storage
      // che sopravvive fra invocazioni. Prima il ramo "fuori range" non
      // scriveva lr/ld/r, quindi esisteva un cammino in cui la variabile
      // conservava il valore precedente -> Quartus segnalava un latch
      // potenziale (nel netlist non veniva poi inferito, ma sul sorgente
      // l'avviso e' corretto). Calcolando sempre e scartando nel ramo fuori
      // range il risultato e' identico e nessun cammino lascia una temporanea
      // non scritta.
      // NB: con src >= PROG_SIZE la sottrazione va in wrap a 16 bit, ma quel
      // valore e' scartato dal ramo che ritorna 0.
      lr = PROG_SIZE - src;
      ld = DISP_SIZE - {1'b0, dst};
      r  = {1'b0, n};
      if (lr < {7'd0, r})         r = lr[8:0];
      if ({3'd0, ld} < {7'd0, r}) r = ld[8:0];
      cp_limit = (src >= PROG_SIZE || {1'b0, dst} >= DISP_SIZE) ? 9'd0 : r;
    end
  endfunction

  // count per la func 2 (RIEMPIMENTO): Stella NON controlla ROMdata, perche'
  // la func 2 non legge la ROM - param[0] e' il valore da scrivere e
  // param[1] non e' un indirizzo (CartDPCPlus.cxx:206-215, il solo confronto
  // e' `destBase < myDisplayImage.size()`). Usando cp_limit anche qui, un
  // param[1]:param[0] >= 29696 - cioe' quasi qualunque valore di riempimento
  // con byte alto >= 0x74 - faceva tornare 0 e la CALLFUNCTION 2 non
  // riempiva NULLA, in silenzio.
  function [8:0] cp_limit_fill;
    input [11:0] dst; input [7:0] n;
    reg [12:0] ld; reg [8:0] r;
    begin
      // stessa forma di cp_limit: assegnazioni incondizionate, niente 10776
      ld = DISP_SIZE - {1'b0, dst};
      r  = {1'b0, n};
      if ({3'd0, ld} < {7'd0, r}) r = ld[8:0];
      cp_limit_fill = ({1'b0, dst} >= DISP_SIZE) ? 9'd0 : r;
    end
  endfunction

  //--------------------------------------------------------------------------
  // Motore di copia CALLFUNCTION func 1/2 (CartDPCPlus.cxx:190-220).
  // Logica di cartuccia pura: NON usa l'ARM.
  //--------------------------------------------------------------------------
  reg        cp_busy, cp_isrom;        // isrom: 1=func1 (ROM->disp), 0=func2 (fill)
  reg [15:0] cp_src;                   // ROMdata
  reg [11:0] cp_dst;                   // destBase
  reg [8:0]  cp_len, cp_rd, cp_wr;     // len<=255
  reg [7:0]  cp_val;                   // param[0] per func2

  //--------------------------------------------------------------------------
  // STEP 1 - GENERATORE DI NUMERI CASUALI (CartDPCPlus.cxx:147-161).
  // LFSR a 32 bit, clockabile in AVANTI (RANDOM0NEXT) e all'INDIETRO
  // (RANDOM0PRIOR). Seed "DPC+" = 0x2B435044 (CartDPCPlus.cxx:122).
  // Prima era del tutto assente: le letture tornavano 0 fisso e le scritture
  // cadevano nel default. 32 ROM su 37 lo inizializzano ($1071 = RWRITE1).
  //   avanti:   rot_dx(x,11) ^ (x[10] ? 0x10adab1e : 0)
  //   indietro: x[31] ? rot_sx(0x10adab1e^x, 11) : rot_sx(x, 11)
  //--------------------------------------------------------------------------
  reg  [31:0] rng;
  wire [31:0] rng_x    = 32'h10adab1e ^ rng;
  wire [31:0] rng_next = (rng[10] ? 32'h10adab1e : 32'd0) ^ {rng[10:0], rng[31:11]};
  wire [31:0] rng_prev = rng[31] ? {rng_x[20:0], rng_x[31:21]}
                                 : {rng[20:0],   rng[31:21]};

  //--------------------------------------------------------------------------
  // STEP 4 - MOTORE MUSICALE DPC+ A 3 CANALI.
  //
  // Era interamente stub: AMPLITUDE tornava 0 fisso, WAVEFORM e NOTE non
  // facevano nulla -> i titoli che usano l'audio DPC+ erano MUTI (si sentivano
  // solo gli effetti generati direttamente dalla TIA).
  //
  // Stella (CartDPCPlus.cxx:164-179, 315-325, 514, 568):
  //   OSC a 20 kHz; ad ogni tick  music_cnt[x] += music_freq[x]
  //   WAVEFORMx (poke fn6 idx5-7) : music_wave[x] = value & 0x7F
  //   NOTEx     (poke fn9 idx5-7) : music_freq[x] = dword LE letta dalla
  //                                 tabella frequenze (DPCRAM + 7KB) a value*4
  //   AMPLITUDE (peek fn0 idx5)   : somma di 3 byte di Display Data,
  //                                 disp[(wave[x]<<5) + (cnt[x]>>27)]
  //
  // SCELTA: l'ampiezza e' PRECALCOLATA in background invece che al volo. Le
  // tre letture (piu' le quattro di un caricamento NOTE) non entrerebbero nel
  // ciclo di lettura del 6507, e i contatori cambiano solo ad ogni tick OSC -
  // cioe' ogni 716 cicli clk_sys, tempo abbondante. Il servizio gira negli
  // stessi cicli idle usati dal motore di copia, con priorita' alla copia.
  //--------------------------------------------------------------------------
  localparam [12:0] FREQ_BASE = 13'd7168;      // myFrequencyImage = DPCRAM + 7KB
  localparam [24:0] OSC_DIV   = 25'd14318181;  // clk_sys
  localparam [24:0] OSC_INC   = 25'd20000;     // OSC DPC+

  reg [6:0]  mus_wave [0:2];
  reg [31:0] mus_freq [0:2];
  reg [31:0] mus_cnt  [0:2];
  reg [7:0]  mus_amp;                 // AMPLITUDE pronta per la lettura
  reg [24:0] osc_acc;
  wire       osc_tick = (osc_acc >= OSC_DIV);

  // servizio background: job 0 = ricalcolo ampiezza, job 1 = caricamento NOTE
  // 6 ago 2026: tolto `mus_job`, assegnato e mai letto (warning 10036): il
  // servizio background ha un solo tipo di lavoro, la distinzione non esiste.
  reg        mus_busy;
  reg [1:0]  mus_ch;                  // canale 0..2
  reg [2:0]  mus_ph;                  // fase lettura M10K
  reg [7:0]  mus_sum;                 // accumulatore ampiezza
  reg [31:0] mus_tmp;                 // dword NOTE in composizione
  reg [1:0]  mus_nb;                  // byte NOTE 0..3
  reg [7:0]  mus_nval;                // valore scritto in NOTEx
  reg        note_pend;               // NOTE da caricare
  reg [1:0]  note_ch;

  wire [11:0] amp_idx = {mus_wave[mus_ch], 5'd0} + {7'd0, mus_cnt[mus_ch][31:27]};

  //--------------------------------------------------------------------------
  // FSM principale
  //--------------------------------------------------------------------------
  localparam [2:0] S_IDLE=3'd0, S_R1=3'd1, S_R2=3'd2, S_RD=3'd3,
                   S_M1=3'd4,   S_M2=3'd5, S_MD=3'd6;
  reg [2:0] state;
  // 6 ago 2026: `pend_idx` RESTA. Quartus lo segnalava come "assegnato e mai
  // letto" (10036) perche' l'unico lettore e' il TESTBENCH, che il tool non
  // vede: la sonda conta le letture DPC+ per fetcher e per funzione, ed e' cio'
  // che permette di individuare uno stream rotto. Toglierlo avrebbe fatto
  // sparire un warning al prezzo di una capacita' diagnostica reale.
  // Il warning si chiude portandolo su un'uscita di debug gia' esistente
  // (stesso rimedio gia' usato in cdf_bridge.sv per `lda_valid`).
  reg [2:0] pend_fn, pend_idx;
  reg [7:0] pend_flag;

  // flag di windowing sull'indice corrente (CartDPCPlus.cxx:277)
  // indirizzo effettivo dopo eventuale redirect Fast Fetch
  // ORDINE CORRETTO (Stella CartDPCPlus.cxx:277-283): il redirect Fast Fetch
  // SOSTITUISCE l'indirizzo PRIMA del confronto con 0x28, non dopo. Qui la
  // priorita' era invertita: un accesso gia' dentro $1000-$1027 con LDAimm
  // attivo usava il proprio offset invece del byte di programma.
  wire       ff_redir = fast_fetch && lda_imm && (rom_q < 8'h28);
  wire [7:0] eff_addr = ff_redir ? rom_q : off_l[7:0];
  wire       eff_isreg = ff_redir || (off_l < 12'h028);
  wire [2:0] eff_idx  = eff_addr[2:0];
  wire [2:0] eff_fn   = eff_addr[5:3];

  integer i;
  always @(posedge clk) begin
    ram_we_a <= 1'b0;
    callfn   <= 1'b0;

    if (rst) begin
      state <= S_IDLE; addr_d <= 16'hFFFF; rwn_d <= 1'b1; bank_reg <= 3'd5;
      fast_fetch <= 1'b0; lda_imm <= 1'b0; param_ptr <= 4'd0;
      m6502_dout <= 8'd0; cp_busy <= 1'b0;
      rng <= 32'h2B435044;                      // STEP 1: seed "DPC+"
      osc_acc <= 25'd0; mus_amp <= 8'd0; mus_busy <= 1'b0; note_pend <= 1'b0;
      mus_ph <= 3'd0; mus_ch <= 2'd0; mus_sum <= 8'd0;
      mus_nb <= 2'd0; mus_tmp <= 32'd0; mus_nval <= 8'd0; note_ch <= 2'd0;
      for (i = 0; i < 3; i = i + 1) begin
        mus_wave[i] <= 7'd0; mus_freq[i] <= 32'd0; mus_cnt[i] <= 32'd0;
      end
      for (i = 0; i < 8; i = i + 1) begin
        tops[i] <= 8'd0; bots[i] <= 8'd0; cnt[i] <= 12'd0;
        fcnt[i] <= 20'd0; finc[i] <= 8'd0;
      end
    end else begin
      // STEP 4: OSC 20 kHz. Ad ogni tick avanzano i contatori musicali e si
      // richiede il ricalcolo dell'ampiezza.
      if (osc_tick) begin
        osc_acc <= osc_acc - OSC_DIV + OSC_INC;
        for (i = 0; i < 3; i = i + 1) mus_cnt[i] <= mus_cnt[i] + mus_freq[i];
        if (!mus_busy) begin
          mus_busy <= 1'b1;
          mus_ch <= 2'd0; mus_ph <= 3'd0; mus_sum <= 8'd0;
        end
      end else osc_acc <= osc_acc + OSC_INC;

      if (trigger) begin addr_d <= m6502_addr; rwn_d <= m6502_rwn; end
      // una lettura del 6507 requisisce ram_addr_a: la lettura musicale in
      // corso va ripetuta dall'inizio (stessa logica del motore di copia).
      if (trigger && mus_busy) mus_ph <= 3'd0;

      //---------------------------------------------------------------- READ
      if (trigger && m6502_rwn) begin
        // Stella legge SEMPRE il byte programma per primo (peekvalue), anche
        // quando l'accesso e' a un registro: il redirect Fast Fetch dipende
        // dal suo valore. La copia in corso viene sospesa (riparte da cp_wr).
        off_l      <= off;
        rom_addr_r <= PROG_BASE + {bank_reg, 12'd0} + {3'd0, off};
        if (cp_busy) cp_rd <= cp_wr;
        state <= S_R1;
      end
      //--------------------------------------------------------------- WRITE
      else if (trigger && !m6502_rwn) begin
        if (is_reg_w) begin
          case (w_func)
            // DFxFRACLOW: (fcnt & myFractionalLowMask) | (value << 8).
            //   Stable  0x0F0000 -> {fcnt[19:16], value, 8'd0}
            //   Jitter  0x0F00FF -> {fcnt[19:16], value, fcnt[7:0]}
            // La maschera dipende dal driver (vedi il blocco CRC piu' sopra).
            4'd0: fcnt[w_index] <= frac_stable
                    ? {fcnt[w_index][19:16], m6502_din, 8'd0}
                    : {fcnt[w_index][19:16], m6502_din, fcnt[w_index][7:0]};
            4'd1: fcnt[w_index] <= {m6502_din[3:0], fcnt[w_index][15:0]};
            4'd2: begin finc[w_index] <= m6502_din;
                        fcnt[w_index] <= {fcnt[w_index][19:8], 8'd0}; end
            4'd3: tops[w_index] <= m6502_din;
            4'd4: bots[w_index] <= m6502_din;
            4'd5: cnt[w_index]  <= {cnt[w_index][11:8], m6502_din};
            4'd6: case (w_index)
                    3'd0: fast_fetch <= (m6502_din == 8'd0);
                    3'd1: if (param_ptr < 4'd8) begin
                            param[param_ptr[2:0]] <= m6502_din;
                            param_ptr <= param_ptr + 4'd1;
                          end
                    3'd2: begin   // CALLFUNCTION
                            // Stella azzera myParameterPointer SOLO nei casi
                            // 0/1/2 (CartDPCPlus.cxx:189,204,219); i casi
                            // 254/255 (chiamata ARM) e il default lo lasciano
                            // INTATTO. Qui era azzerato per tutti i valori.
                            case (m6502_din)
                              8'd0: param_ptr <= 4'd0;      // solo reset ptr
                              8'd1, 8'd2: begin
                                // il reset del puntatore in Stella avviene
                                // SEMPRE per func 1/2, anche quando la copia
                                // non parte: non va messo sotto !cp_busy.
                                param_ptr <= 4'd0;
                                if (!cp_busy) begin
                                  cp_isrom <= (m6502_din == 8'd1);
                                  cp_src   <= {param[1], param[0]};
                                  cp_dst   <= cnt[param[2][2:0]];
                                  cp_val   <= param[0];
                                  cp_rd    <= 9'd0; cp_wr <= 9'd0;
                                  cp_len   <= (m6502_din == 8'd1)
                                    ? cp_limit({param[1], param[0]},
                                               cnt[param[2][2:0]], param[3])
                                    : cp_limit_fill(cnt[param[2][2:0]], param[3]);
                                  cp_busy  <= 1'b1;
                                end
                              end
                              8'hFE, 8'hFF: begin           // ARM (PASSO 2)
                                callfn <= 1'b1; callfn_val <= m6502_din;
                              end
                              default: ;
                            endcase
                          end
                    3'd5, 3'd6, 3'd7:                       // STEP 4: WAVEFORM
                      mus_wave[w_index - 3'd5] <= m6502_din[6:0];
                    default: ;                              // 3/4 riservati
                  endcase
            4'd7: begin                                     // DFxPUSH
                    cnt[w_index] <= cnt[w_index] - 12'd1;
                    ram_addr_a  <= DISP_BASE + {1'b0, cnt[w_index] - 12'd1};
                    ram_wdata_a <= m6502_din; ram_we_a <= 1'b1;
                  end
            4'd8: cnt[w_index] <= {m6502_din[3:0], cnt[w_index][7:0]};
            4'd9: case (w_index)                            // STEP 1: RNG
                    3'd0: rng <= 32'h2B435044;                        // RRESET
                    3'd1: rng <= {rng[31:8],  m6502_din};             // RWRITE0
                    3'd2: rng <= {rng[31:16], m6502_din, rng[7:0]};   // RWRITE1
                    3'd3: rng <= {rng[31:24], m6502_din, rng[15:0]};  // RWRITE2
                    3'd4: rng <= {m6502_din,  rng[23:0]};             // RWRITE3
                    3'd5, 3'd6, 3'd7: begin                 // STEP 4: NOTE0-2
                      note_pend <= 1'b1;                    // caricamento dword
                      note_ch   <= w_index[1:0] - 2'd1;     // idx5->0,6->1,7->2
                      mus_nval  <= m6502_din;
                      mus_nb    <= 2'd0;
                      // il NOTE ha priorita': un eventuale calcolo ampiezza in
                      // corso riparte pulito dopo, non a meta'.
                      mus_ph    <= 3'd0; mus_ch <= 2'd0; mus_sum <= 8'd0;
                    end
                    default: ;
                  endcase
            4'd10: begin                                    // DFxWRITE
                    ram_addr_a  <= DISP_BASE + {1'b0, cnt[w_index]};
                    ram_wdata_a <= m6502_din; ram_we_a <= 1'b1;
                    cnt[w_index] <= cnt[w_index] + 12'd1;
                  end
            default: ;
          endcase
          if (cp_busy) cp_rd <= cp_wr;
          state <= S_IDLE;
        end else begin
          if (is_hot) bank_reg <= off[2:0] - 3'd6;
          state <= S_IDLE;
        end
      end
      //-------------------------------------------------------------- STATES
      else begin
        case (state)
          S_R1: state <= S_R2;
          S_R2: state <= S_RD;
          S_RD: begin
            lda_imm <= 1'b0;                    // Stella: sempre azzerato
            if (eff_isreg) begin
              pend_idx <= eff_idx; pend_fn <= eff_fn;
              case (eff_fn)
                3'd1, 3'd2: begin               // DFxDATA / DFxDATAW
                  ram_addr_a <= DISP_BASE + {1'b0, cnt[eff_idx]};
                  // Stella calcola il flag con il contatore PRE-incremento
                  // (CartDPCPlus.cxx:277 prima del case): va latchato qui.
                  pend_flag <= (((tops[eff_idx] - cnt[eff_idx][7:0]) >
                                 (tops[eff_idx] - bots[eff_idx])) ? 8'hFF : 8'h00);
                  cnt[eff_idx] <= cnt[eff_idx] + 12'd1;
                  state <= S_M1;
                end
                3'd3: begin                     // DFxFRACDATA
                  ram_addr_a <= DISP_BASE + {1'b0, fcnt[eff_idx][19:8]};
                  fcnt[eff_idx] <= fcnt[eff_idx] + {12'd0, finc[eff_idx]};
                  state <= S_M1;
                end
                3'd4: begin                     // DFxFLAG (indici 0..3)
                  m6502_dout <= (eff_idx < 3'd4) ?
                    (((tops[eff_idx] - cnt[eff_idx][7:0]) >
                      (tops[eff_idx] - bots[eff_idx])) ? 8'hFF : 8'h00) : 8'd0;
                  state <= S_IDLE;
                end
                3'd0: begin                     // STEP 1: RANDOM / AMPLITUDE
                  // Stella clocca PRIMA e restituisce il valore aggiornato
                  // (CartDPCPlus.cxx:286-296).
                  case (eff_idx)
                    3'd0: begin rng <= rng_next; m6502_dout <= rng_next[7:0]; end
                    3'd1: begin rng <= rng_prev; m6502_dout <= rng_prev[7:0]; end
                    3'd2: m6502_dout <= rng[15:8];
                    3'd3: m6502_dout <= rng[23:16];
                    3'd4: m6502_dout <= rng[31:24];
                    3'd5: m6502_dout <= mus_amp;  // STEP 4: AMPLITUDE
                    default: m6502_dout <= 8'd0;  // 6/7 riservati
                  endcase
                  state <= S_IDLE;
                end
                default: begin                  // fn 5/6/7: non usate
                  m6502_dout <= 8'd0;
                  state <= S_IDLE;
                end
              endcase
            end else begin
              m6502_dout <= rom_q;              // byte programma
              if (is_hot_l) bank_reg <= off_l[2:0] - 3'd6;
              if (fast_fetch) lda_imm <= (rom_q == 8'hA9);
              state <= S_IDLE;
            end
          end
          S_M1: state <= S_M2;
          S_M2: state <= S_MD;
          S_MD: begin
            m6502_dout <= (pend_fn == 3'd2) ? (ram_q_a & pend_flag) : ram_q_a;
            state <= S_IDLE;
          end
          //-------------------------------------------------- copia in background
          default: begin
          if (cp_busy) begin
            // pipeline: indirizzo ROM emesso a cp_rd, dato disponibile 3 clk
            // dopo -> si scrive cp_wr quando cp_rd >= cp_wr+3 (func1); func2
            // non legge la ROM e scrive 1 byte per ciclo.
            if (cp_isrom) begin
              // BUG CORRETTO (1 ago 2026) - la copia perdeva gli ULTIMI 3 BYTE
              // e restava appesa per sempre.
              // Prima cp_rd si fermava a cp_len: da quel momento la condizione
              // di scrittura (cp_rd >= cp_wr+3) non poteva piu' essere vera per
              // gli ultimi byte in volo, cp_wr si bloccava a cp_len-2 e
              // `if (cp_wr >= cp_len) cp_busy <= 0` non scattava MAI.
              // Con cp_busy incastrato a 1, il guardiano `if (!cp_busy)` sul
              // CALLFUNCTION scartava in silenzio TUTTE le copie successive:
              // dopo la primissima, la Display Data non veniva piu' aggiornata.
              // DK Arcade emette una CALLFUNCTION 1 ogni ~70 cicli CPU.
              // Ora cp_rd continua a contare fino a cp_len+3 (svuotamento della
              // pipeline) mentre l'INDIRIZZO si ferma a cp_len-1: i tre byte
              // ancora in volo vengono scritti nei tre cicli successivi.
              if (cp_rd < cp_len + 9'd3) cp_rd <= cp_rd + 9'd1;
              if (cp_rd < cp_len)
                rom_addr_r <= PROG_BASE + cp_src[14:0] + {6'd0, cp_rd};
              if ((cp_rd >= cp_wr + 9'd3) && (cp_wr < cp_len)) begin
                ram_addr_a  <= DISP_BASE + {1'b0, cp_dst} + {4'd0, cp_wr};
                ram_wdata_a <= rom_q; ram_we_a <= 1'b1;
                cp_wr <= cp_wr + 9'd1;
              end
            end else begin
              if (cp_wr < cp_len) begin
                ram_addr_a  <= DISP_BASE + {1'b0, cp_dst} + {4'd0, cp_wr};
                ram_wdata_a <= cp_val; ram_we_a <= 1'b1;
                cp_wr <= cp_wr + 9'd1;
              end
            end
            if (cp_wr >= cp_len) cp_busy <= 1'b0;
          end
          //----------------------------------- STEP 4: servizio musicale
          // Gira solo quando la copia non sta usando la porta B. Priorita' al
          // caricamento NOTE (evento raro e puntuale), poi il ricalcolo
          // dell'ampiezza, che ha 716 cicli di margine fino al tick successivo.
          else if (note_pend) begin
            case (mus_ph)
              3'd0: begin
                ram_addr_a <= FREQ_BASE + {3'd0, mus_nval, 2'd0} + {11'd0, mus_nb};
                mus_ph <= 3'd1;
              end
              3'd3: begin
                case (mus_nb)
                  2'd0: mus_tmp[7:0]   <= ram_q_a;
                  2'd1: mus_tmp[15:8]  <= ram_q_a;
                  2'd2: mus_tmp[23:16] <= ram_q_a;
                  default: begin                     // 4o byte: dword completa
                    mus_freq[note_ch] <= {ram_q_a, mus_tmp[23:0]};
                    note_pend <= 1'b0;
                  end
                endcase
                mus_nb <= mus_nb + 2'd1;
                mus_ph <= 3'd0;
              end
              default: mus_ph <= mus_ph + 3'd1;
            endcase
          end
          else if (mus_busy) begin
            case (mus_ph)
              3'd0: begin
                ram_addr_a <= DISP_BASE + {1'b0, amp_idx};
                mus_ph <= 3'd1;
              end
              3'd3: begin
                if (mus_ch == 2'd2) begin
                  mus_amp  <= mus_sum + ram_q_a;     // somma dei 3 canali
                  mus_busy <= 1'b0;
                end else begin
                  mus_sum <= mus_sum + ram_q_a;
                  mus_ch  <= mus_ch + 2'd1;
                end
                mus_ph <= 3'd0;
              end
              default: mus_ph <= mus_ph + 3'd1;
            endcase
          end
          end
        endcase
      end
    end
  end

endmodule

//----------------------------------------------------------------------------
// ROM 32KB, M10K true DUAL-CLOCK a LARGHEZZA MISTA (2 agosto 2026).
//   porta A = clk_a (clk_vid): fetch ARM, **32 bit** -> 8192 word.
//   porta B = clk_b (clk_sys): engine 6502 read + loader write, 8 bit.
//
// PERCHE' 32 BIT SU A: prima entrambe le porte erano a 8 bit, quindi ogni word
// ARM andava composta camminando 4 lane byte (contatore arm_ph) = 6 cicli
// clk_vid per lettura contro i 2 del cdf_bridge, che le sue M10K le ha gia' a
// 32 bit (cdf_m10k_memories.sv:33, harmony_m10k_tdp.sv:59). MISURATO il 1-2 ago
// su tutte e 6 le ROM DPC+ rotte: dal 19% (Meooow 2) al 56% (DK Arcade) delle
// scritture ARM in Display Data arrivavano con tia_vblank=0, cioe' mentre la
// TIA disegnava gia' la parte visibile.
//
// L'indirizzo di porta A e' ora un indirizzo di WORD: bus_addr[14:2]. E'
// sicuro perche' thumb_core allinea SEMPRE bus_addr (`& 32'hFFFFFFFC` su ogni
// LDR/STR, e {pc[31:2],2'b00} sul fetch): i byte passano da bus_be, mai da un
// indirizzo disallineato. La mappatura lane e' little-endian - il byte
// all'indirizzo 4W+i sta nei bit [8i +: 8] della word W - che e' esattamente
// quella del device reale e quella che il vecchio cammino a lane produceva.
//
// Schema identico a harmony_m10k_tdp: TUTTI i _reg_b su CLOCK1, entrambi i
// clock connessi. Mescolare default e override sui registri di porta B fa
// scattare gli errori Intel 272006/287078/12152 (in BIDIR_DUAL_PORT il default
// dei _reg_b e' CLOCK1: lasciarne uno a default con un solo clock connesso lo
// aggancia a un clock1 inesistente). L'ARM non scrive mai la ROM -> wren_a=0.
//----------------------------------------------------------------------------
module dpcp_rom_m10k #(parameter INIT_FILE = "")(
  input  wire        clk_a,          // clk_vid (ARM read, 32 bit)
  input  wire [12:0] addr_a,         // indirizzo di WORD (bus_addr[14:2])
  output wire [31:0] q_a,
  input  wire        clk_b,          // clk_sys (engine 6502 + loader)
  input  wire [14:0] addr_b,
  input  wire [7:0]  data_b,
  input  wire        wren_b,
  output wire [7:0]  q_b
);
  altsyncram #(
    .intended_device_family("Cyclone V"), .lpm_type("altsyncram"),
    .operation_mode("BIDIR_DUAL_PORT"), .ram_block_type("M10K"),
    .width_a(32), .widthad_a(13), .numwords_a(8192),  .width_byteena_a(1),
    .width_b(8),  .widthad_b(15), .numwords_b(32768), .width_byteena_b(1),
    // 4 agosto 2026 - stessa ottimizzazione di latenza gia' fatta sul percorso
    // CDF (cdf_m10k_memories.sv / harmony_m10k_tdp.sv): senza il registro di
    // uscita la porta A ha 1 ciclo di latenza invece di 2, e una lettura ARM
    // costa 2 cicli di S_MEM invece di 3. Sul CDF valeva -19,6% di cicli ARM su
    // Elevator Agent; qui serve DK Arcade e gli altri DPC+, che finora erano
    // rimasti indietro perche' hanno memorie e handshake propri.
    .outdata_reg_a("UNREGISTERED"),
    .outdata_reg_b("CLOCK1"), .address_reg_b("CLOCK1"), .indata_reg_b("CLOCK1"),
    .wrcontrol_wraddress_reg_b("CLOCK1"), .byteena_reg_b("CLOCK1"),
    .read_during_write_mode_port_a("DONT_CARE"),
    .read_during_write_mode_port_b("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized("FALSE"), .init_file(INIT_FILE)
  ) ram_i (
    .clock0(clk_a), .clock1(clk_b),
    .address_a(addr_a), .data_a(32'd0), .byteena_a(1'b1), .wren_a(1'b0),  .q_a(q_a),
    .address_b(addr_b), .data_b(data_b), .byteena_b(1'b1), .wren_b(wren_b), .q_b(q_b),
    .aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0),
    .clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a(1'b1), .rden_b(1'b1), .eccstatus()
  );
endmodule

//----------------------------------------------------------------------------
// RAM 8KB byte-wide dedicata DPC+, M10K true DUAL-CLOCK (STEP A).
//   porta A = clk_a (clk_vid): read/write ARM + init dal loader su porta B.
//   porta B = clk_b (clk_sys): engine 6502 (fetcher/copia) e init al download.
// L'ARM SCRIVE la Display Data, quindi qui porta A e' read-write (a differenza
// della ROM). Stesso schema _reg_b/CLOCK1 di harmony_m10k_tdp.
//
// NOTA 2a - read_during_write_mode_mixed_ports("DONT_CARE").
// Con la porta A viva (STEP C) esiste ora una collisione mixed-port reale:
// l'ARM scrive Display Data su clk_vid mentre il motore 6502 la legge su
// clk_sys. VERIFICATO: non e' una scelta, e' l'unico modo disponibile - su
// Cyclone V in BIDIR_DUAL_PORT con due clock indipendenti non esiste un
// comportamento read-during-write mixed-port definito, e infatti la Harmony
// RAM del motore CDF - che gira su hardware da sempre - usa esattamente lo
// stesso attributo (harmony_m10k_tdp.sv:68-70). Portare i tre attributi a
// parita' con il riferimento gia' funzionante e' quindi la scelta corretta.
// La mitigazione e' a livello di PROTOCOLLO, non di attributo. Con la porta A
// a 32 bit (2 ago) la finestra si stringe ancora: una word ARM e' scritta in UN
// solo ciclo invece che in quattro fasi consecutive.
//
// LARGHEZZA MISTA (2 agosto 2026): porta A a 32 bit -> 2048 word, porta B a
// 8 bit -> 8192 byte. Vedi la nota estesa su dpcp_rom_m10k. Qui la porta A
// SCRIVE, quindi servono i byte enable: width_byteena_a(4) pilotato da bus_be,
// con bus_wdata gia' replicato da thumb_core per STRB/STRH.
//----------------------------------------------------------------------------
// PERCHE' A BANCHI E NON A LARGHEZZA MISTA (misurato, 2 ago 2026).
// Il primo tentativo era una sola altsyncram con width_a(32)/width_b(8) e
// width_byteena_a(4), come la ROM qui sopra. Quartus 17.0 la RIFIUTA:
//   Error (272006): Cannot use port A width with port B width in altsyncram
//   Error (287078)/(12152) a cascata
// La ROM (sola lettura, width_byteena_a(1)) passa invece senza problemi:
// il vincolo del Cyclone V non e' il rapporto 4:1 ma i BYTE ENABLE sulla porta
// larga in modalita' mista.
//
// Qui i byte enable servono davvero - l'ARM fa STRB/STRH nella Display Data
// mentre disegna - e un read-modify-write costerebbe proprio sul carico che si
// sta cercando di accelerare. Quindi: 4 banchi da 2048x8, uno per lane.
//   porta A: tutti allo stesso indirizzo di word, wren_a & be_a[i] -> il byte
//            enable diventa un wren separato, gratis. Lettura: 4 byte in
//            parallelo = una word in un solo accesso.
//   porta B: indirizzo di word addr_b[12:2] su tutti, lane addr_b[1:0] che
//            decodifica il wren e multiplexa la lettura.
// Stessi bit totali, stessa latenza (2 cicli su entrambe le porte), e le
// primitive sono identiche a quelle che gia' passavano il fit.
//
// Il selettore di lettura di porta B va RITARDATO DI DUE CICLI per allinearsi
// al dato: la M10K ha indirizzo registrato + uscita registrata, quindi q_b
// corrisponde all'indirizzo presentato due fronti prima.
module dpcp_ram_m10k #(parameter INIT_FILE = "")(
  input  wire        clk_a,          // clk_vid (ARM, 32 bit)
  input  wire [10:0] addr_a,         // indirizzo di WORD (bus_addr[12:2])
  input  wire [31:0] data_a,
  input  wire [3:0]  be_a,           // byte enable = bus_be
  input  wire        wren_a,
  output wire [31:0] q_a,
  input  wire        clk_b,          // clk_sys (engine 6502)
  input  wire [12:0] addr_b,         // indirizzo di BYTE
  input  wire [7:0]  data_b,
  input  wire        wren_b,
  output wire [7:0]  q_b
);
  wire [31:0] qa_all, qb_all;       // {lane3, lane2, lane1, lane0}
  assign q_a = qa_all;

  reg [1:0] selb1, selb2;
  always @(posedge clk_b) begin selb1 <= addr_b[1:0]; selb2 <= selb1; end
  assign q_b = qb_all[selb2*8 +: 8];

  genvar g;
  generate for (g = 0; g < 4; g = g + 1) begin : lane
    altsyncram #(
      .intended_device_family("Cyclone V"), .lpm_type("altsyncram"),
      .operation_mode("BIDIR_DUAL_PORT"), .ram_block_type("M10K"),
      .width_a(8), .widthad_a(11), .numwords_a(2048), .width_byteena_a(1),
      .width_b(8), .widthad_b(11), .numwords_b(2048), .width_byteena_b(1),
      // uscita porta A non registrata: vedi la nota in dpcp_rom_m10k
      .outdata_reg_a("UNREGISTERED"),
      .outdata_reg_b("CLOCK1"), .address_reg_b("CLOCK1"), .indata_reg_b("CLOCK1"),
      .wrcontrol_wraddress_reg_b("CLOCK1"), .byteena_reg_b("CLOCK1"),
      .read_during_write_mode_port_a("DONT_CARE"),
      .read_during_write_mode_port_b("DONT_CARE"),
      .read_during_write_mode_mixed_ports("DONT_CARE"),
      .power_up_uninitialized("FALSE"), .init_file(INIT_FILE)
    ) ram_i (
      .clock0(clk_a), .clock1(clk_b),
      .address_a(addr_a),      .data_a(data_a[g*8 +: 8]), .byteena_a(1'b1),
      .wren_a(wren_a & be_a[g]),  .q_a(qa_all[g*8 +: 8]),
      .address_b(addr_b[12:2]), .data_b(data_b), .byteena_b(1'b1),
      .wren_b(wren_b & (addr_b[1:0] == g[1:0])), .q_b(qb_all[g*8 +: 8]),
      .aclr0(1'b0), .aclr1(1'b0), .addressstall_a(1'b0), .addressstall_b(1'b0),
      .clocken0(1'b1), .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
      .rden_a(1'b1), .rden_b(1'b1), .eccstatus()
    );
  end endgenerate
endmodule
`default_nettype wire
