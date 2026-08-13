// k7800 (c) by Jamie Blanks

// k7800 is licensed under a
// Creative Commons Attribution-NonCommercial 4.0 International License.

// You should have received a copy of the license along with this
// work. If not, see http://creativecommons.org/licenses/by-nc/4.0/.

module cart2600
(
	// Physical Pins
	output logic [7:0] d_out, // Data bus
	input    [7:0]  d_in,  // Data bus
	input    [12:0] a_in,  // Address bus
	input           rwn,   // 6502 R/W_n (1 = read cycle)

	// Helpers
	input           clk,        // Master Clock
	input           clk_vid,    // Phase B: ARM Thumb core domain (~57.3 MHz, PLL 4x clk)
	input           reset,      // System warm reset
	input           ce,         // Original system clock enable (~3.579mhz) used to divide into crystals
	input           phi1,       // CPU Phase 1 Signal (used for FE to catch data at the right moment)
	output   [7:0]  oe,         // Output Enable mask
	input    [7:0]  open_bus,   // Input open bus to use when not driving data bus (Obselete, use oe)

	// Autodetect info
	input           sc,         // Superchip Enable
	input    [4:0]  mapper,     // Bankswitching type (ie Mapper)
	input    [1:0]  cdf_family, // CDF sub-version from detect2600: 00=CDF0/1 01=CDFJ 10=CDFJ+
	input           arm_enable, // Master enable for generic ARM Thumb / CDF subsystem
	input           cart_download,
	input    [18:0] ioctl_addr,
	input    [7:0]  ioctl_dout,
	input           ioctl_wr,
	
	// SDRAM ROM storage interface
	input    [7:0]  rom_do,     // Incoming ROM data from the sdram
	input   [18:0]  rom_size,   // Full rom size for address masking
	output  [18:0]  rom_a,      // Outgoing absolute rom address for image.
	output          rom_read,   // Initiate read from SDRAM
	
	output   [17:0] cartram_addr,
	output          cartram_wr,
	output          cartram_rd,
	output   [7:0]  cartram_wrdata,
	input    [7:0]  cartram_data,

	// Tape Signals
	output          tape_audio, // Tape audio output
	input    [1:0]  tape_in,    // ADC tape input
	input           fix_sc_cs,  // Fix Supercharger Checksums menu option

	// CDF/DPC+ ARM run in progress: stalls the 6507 like a real Harmony
	// board does (CALLFN is a blocking call on hardware - the driver on
	// $1FF3=254/255 expects results to be ready the instant control
	// returns, not hundreds/thousands of clk cycles later). Without this
	// the 6502 races ahead of the ARM and reads a half-updated Harmony
	// RAM/display image, which shows up as a corrupted (partially stale)
	// frame right where the CDF driver's CALLFN calls become frequent.
	output          arm_cpu_stall,
	// FASE9: wait-state selettivo sul solo serve datastream (vedi top.sv/RDY)
	output          ds_wait,
	// RIPRISTINATA il 2 ago 2026 dopo la misura [armout] (vedi il blocco
	// "STALLO 6507 DURANTE UN RUN ARM DPC+"): senza di essa il 6507 rilegge
	// DF0DATA 6 cicli dopo la CALLFUNCTION, prima che l'ARM abbia scritto.
	// E' REGISTRATA (dpcp_stall_r) proprio per la lezione sul fan-out.
	output          dpcp_arm_stall,

	// Livello 3 (ping-pong Display Data): fase Kernel/VBlank (bit VBLANK software
	// dalla TIA), usata dalla logica di switch A/B/C nel subsystem.
	input           vblank_sw
);
	`define NUM_MAPPERS BANKEND

	// Muxxing signals
	logic [18:0] rom_addr[`NUM_MAPPERS];
	logic [7:0] direct_do[`NUM_MAPPERS];
	logic [15:0] flags_out[`NUM_MAPPERS]; // Flag bit 0 is direct_do in use, bit 1 is output enable used;
	logic [7:0]  out_en[`NUM_MAPPERS];
	logic        ram_rw[`NUM_MAPPERS];
	logic        ram_sel[`NUM_MAPPERS];
	logic [17:0] ram_a[`NUM_MAPPERS];
	logic [12:0] old_ain;
	logic [7:0]  bg_data;
	logic        ar_read;
	logic [7:0]  cr_do;


	logic [18:0] sel_rom_addr;
	logic [7:0] sel_direct_do;
	logic [15:0] sel_flags_out;
	logic [7:0]  sel_out_en;
	logic        sel_ram_rw;
	logic        sel_ram_sel;
	logic [17:0] sel_ram_a;
	logic [18:0] rom_mask;

	wire address_change = old_ain != a_in;
	logic [7:0]  arm_dout;
	logic        arm_busy;
	logic        arm_done;
	logic [18:0] arm_rom_a;
	logic        arm_rom_read;

	assign rom_mask = rom_size - 1'd1;
	// STEP 3 punto 3: rom_read ristretto al SOLO BANKCDF. dpcplus_bridge ha
	// ROM/RAM proprie in M10K e non legge dalla SDRAM, quindi BANKDPCP ricade
	// sul default ~address_change come ogni mapper non-ARM.
	assign rom_read = (mapper == BANKAR) ? ar_read :
	                  ((mapper == BANKCDF) && arm_enable) ? arm_rom_read :
	                  ~address_change;
	// STEP 3 punto 4: due termini SEPARATI e qualificati per mapper. Entrambi
	// i motori richiedono l'ARM abilitato da OSD, quindi la condizione e'
	// identica nei due rami, ma resta scritta distinta per non riunire di
	// nuovo CDF e DPC+ in un unico wire di controllo.
	wire is_bad_game = (~arm_enable & (mapper == BANKCDF))
	                 | (~arm_enable & (mapper == BANKDPCP));

	// Handle unsupportable ARM mappers if explicitly disabled by user :(
	spram #(.addr_width(11), .mem_init_file("ooo.mif")) badgame_ram
	(
		.clock      (clk),
		.address    (a_in[10:0]),
		.wren       (0),
		.q          (bg_data)
	);

	// Instantiate Generic ARM Thumb & CDF/DPCP Subsystem
	wire subsys_ds_wait;
	// STEP B: bus ARM condiviso (dal subsystem al bridge DPC+). Dichiarato qui
	// perche' usato da entrambe le istanze sottostanti.
	wire        arm_rst_vid;
	wire [31:0] arm_bus_addr, arm_bus_wdata, arm_bus_rdata_dpcp;
	wire [3:0]  arm_bus_be;
	wire        arm_bus_we, arm_bus_req_dpcp, arm_bus_ack_dpcp;
	// STEP C: CALLFUNCTION dal bridge DPC+ verso il core ARM condiviso
	wire        dpcp_callfn;
	wire [7:0]  dpcp_callfn_val;
	atari2600_arm_subsystem u_arm_subsystem (
		.clk           (clk),
		.clk_vid       (clk_vid),
		.reset         (reset | (~arm_enable)),
		.arm_enable    (arm_enable),
		.mapper        (mapper),
		.cdf_family    (cdf_family),
		.a_in          (a_in),
		.d_in          (d_in),
		.rwn           (rwn),
		.cart_download (cart_download),
		.ioctl_addr    (ioctl_addr),
		.ioctl_dout    (ioctl_dout),
		.ioctl_wr      (ioctl_wr),
		.rom_do        (rom_do),
		.rom_size      (rom_size),
		.rom_a         (arm_rom_a),
		.rom_read      (arm_rom_read),
		.d_out         (arm_dout),
		.busy          (arm_busy),
		.done          (arm_done),
		.vblank_sw     (vblank_sw),
		.ds_wait       (subsys_ds_wait),
		.audio_mix_out (),
		// STEP B: bus ARM verso dpcplus_bridge
		.arm_rst_vid        (arm_rst_vid),
		.arm_bus_addr       (arm_bus_addr),
		.arm_bus_wdata      (arm_bus_wdata),
		.arm_bus_be         (arm_bus_be),
		.arm_bus_we         (arm_bus_we),
		.arm_bus_req_dpcp   (arm_bus_req_dpcp),
		.arm_bus_addr_pre   (arm_bus_addr_pre),
		.arm_bus_req_pre    (arm_bus_req_pre),
		.arm_bus_rdata_dpcp (arm_bus_rdata_dpcp),
		.arm_bus_ack_dpcp   (arm_bus_ack_dpcp),
		.dpcp_callfn        (dpcp_callfn),
		.dpcp_callfn_val    (dpcp_callfn_val),
		.dbg_pc        (),
		.dbg_r0        (),
		// sonde del ping-pong: non collegate, ma bastano a far vedere a
		// Quartus che i segnali sono letti (il lettore vero e' il testbench)
		.dbg_pp        ()
	);

	//------------------------------------------------------------------------
	// STEP 3 - motore DPC+ dedicato. Modulo INDIPENDENTE dal motore CDF: non
	// condivide stato con atari2600_arm_subsystem/cdf_bridge (intoccati).
	// Tenuto in reset per ogni mapper diverso da BANKDPCP, cosi' su una
	// cartuccia CDF e' inerte per costruzione (equivalente del core_reset che
	// il subsystem usa per se stesso).
	//------------------------------------------------------------------------
	wire [7:0]  dpcp_dout;
	wire [14:0] dpcp_rom_a;
	wire        dpcp_active = arm_enable && (mapper == BANKDPCP);

	// Indirizzo anticipato dal thumb_core verso il bridge DPC+ (stadio 2).
	wire [31:0] arm_bus_addr_pre;
	wire        arm_bus_req_pre;
	dpcplus_bridge u_dpcplus (
		.clk           (clk),
		.clk_vid       (clk_vid),      // STEP A: solo clock porta ARM delle M10K
		.rst           (reset | ~dpcp_active),
		.m6502_addr    ({3'b000, a_in[12:0]}),
		.m6502_din     (d_in),
		.m6502_rwn     (rwn),
		.m6502_dout    (dpcp_dout),
		.rom_a         (dpcp_rom_a),
		.callfn        (dpcp_callfn),
		.callfn_val    (dpcp_callfn_val),
		.rom_load_we   (cart_download & ioctl_wr & (ioctl_addr < 19'd32768)),
		.rom_load_addr (ioctl_addr[14:0]),
		.rom_load_data (ioctl_dout),
		// STEP B: lato ARM (clk_vid)
		.rst_vid       (arm_rst_vid),
		.bus_addr      (arm_bus_addr),
		.bus_wdata     (arm_bus_wdata),
		.bus_rdata     (arm_bus_rdata_dpcp),
		.bus_be        (arm_bus_be),
		.bus_we        (arm_bus_we),
		.bus_req       (arm_bus_req_dpcp),
		.bus_ack       (arm_bus_ack_dpcp),
		.bus_addr_pre  (arm_bus_addr_pre),
		.bus_req_pre   (arm_bus_req_pre),
		.dbg_bank      (),
		.dbg_ff        ()
	);

	// Only stall the 6507 for the mappers that actually run the ARM (CDF/
	// DPC+); arm_busy stays 0 for every other mapper anyway (subsys_active
	// gate inside atari2600_arm_subsystem), so this is belt-and-braces.
	// STEP 3 punto 1: ristretto al SOLO BANKCDF - arm_busy viene dal subsystem
	// CDF, che con DPC+ e' ora in reset permanente.
	assign arm_cpu_stall = arm_busy && (mapper == BANKCDF) && arm_enable;

	//------------------------------------------------------------------------
	// STALLO 6507 DURANTE UN RUN ARM DPC+  (2 ago 2026) - REINTRODOTTO.
	//
	// Il blocco piu' in basso (rimozione dello STEP 2, luglio) resta come storia, ma la sua
	// premessa era sbagliata. Misura [armout] su 3PointDash:
	//   cyc=9193 CALLFUNCTION 255 dopo 58 push
	//   cyc=9199 prima lettura di DF0DATA: +6 cicli CPU, ARM halted=0
	//   la finestra disp[$01A9..$01BB] e' IDENTICA prima e dopo.
	// Il gioco (codice a $1988) spinge 58 byte, chiama l'ARM e alla PRIMISSIMA
	// istruzione successiva rilegge 19 byte da DF0DATA in zero page $85..$97.
	// Senza stallo rilegge il proprio ingresso invece del risultato, e il frame
	// dopo ricalcola su dati vecchi: il sistema si stabilizza su un punto fisso
	// sbagliato (sintomo "parte centrale dello schermo spostata a sinistra").
	//
	// Stella NON dice che il 6507 puo' osservare l'ARM a meta' lavoro: dice che
	// il run costa zero cicli 6507 (CartDPCPlus.cxx:223-225) perche' e' ATOMICO,
	// cioe' completo prima che il 6507 riprenda. Il nostro ARM impiega tempo
	// vero, quindi delle tre combinazioni possibili
	//   (a) ARM istantaneo, 6507 libero      -> Stella, corretto
	//   (b) ARM lento,      6507 fermato     -> corretto, costa cicli di frame
	//   (c) ARM lento,      6507 libero      -> QUESTA, l'unica che da' dati vecchi
	// eravamo in (c). Si passa a (b), che e' anche quello che il CDF usava.
	//
	// Restrizione a BANKDPCP: il CDF oggi gira su HW SENZA stallo (21 titoli
	// perfetti) e la regressione d'oro Draconian deve restare bit-identica.
	// Il suo idioma e' diverso - non rilegge subito dopo la CALLFUNCTION - e per
	// le corse col Kernel ha gia' l'attesa selettiva ds_wait.
	//
	// REGISTRATO (lezione del blocco sotto, righe 245-254 originali): un termine
	// verso RDY che nasce nel dominio ARM deve attraversare un FF, altrimenti
	// crea un cammino combinatorio da arm_halted_sync fino a RDY a top level e
	// il fitter ripartiziona la duplicazione di `halted` dentro thumb_core
	// (ir->r[] misurato scendere da 60.60 a 57.76 MHz). Un ciclo clk_sys di
	// ritardo (69.8 ns) e' nulla rispetto ai 6 cicli 6507 (5 us) di margine.
	//
	// GUARDIA: un run che non termina non deve piu' poter congelare il 6507 -
	// e' il guasto misurato su HW a luglio (niente VSYNC -> TIA nel ramo di
	// emergenza `&v_count` -> frame ~512 righe, V = 30.70 Hz). 12 clk_sys = 1
	// ciclo 6507, quindi 65535 clk_sys ~ 5460 cicli 6507: oltre 8x il run piu'
	// lungo misurato (615 cicli, 3PointDash frame 1) e sempre meno di un frame
	// intero (19912). Peggior caso: un frame storto, mai un blocco.
	//------------------------------------------------------------------------
	reg [15:0] stall_wd;
	reg        stall_giveup;
	(* preserve *) reg dpcp_stall_r;
	always_ff @(posedge clk) begin
		if (reset || !arm_busy) begin
			stall_wd     <= 16'd0;
			stall_giveup <= 1'b0;
		end else begin
			if (stall_wd != 16'hFFFF) stall_wd <= stall_wd + 16'd1;
			else                      stall_giveup <= 1'b1;
		end
		dpcp_stall_r <= !reset && arm_busy && !stall_giveup && arm_enable &&
		                (mapper == BANKDPCP);
	end
	assign dpcp_arm_stall = dpcp_stall_r;

	// FASE9: il wait selettivo sui datastream e' un meccanismo del SOLO motore
	// CDF (subsys_ds_wait nasce nella FSM S_DWAIT di cdf_bridge). DPC+ non ha
	// datastream ne' S_DWAIT: STEP 3 punto 2 lo restringe a BANKCDF.
	assign ds_wait = subsys_ds_wait && (mapper == BANKCDF) && arm_enable;

	//------------------------------------------------------------------------
	// FIX GLOBALE punto 1 - STALLO 6507 DPC+ (STEP 2): RIMOSSO.
	//
	// Erano presenti qui: `wire is_dpcp_stall_mapper`, il registro
	// `(* preserve *) reg dpcp_stall_r` e `assign dpcp_arm_stall`. Tutti e tre
	// sono stati eliminati, non disattivati: dopo la rimozione del termine da
	// RDY (top.sv) non avevano piu' alcun lettore, e un registro `preserve` che
	// non pilota nulla resta comunque nel fit tenendo vivo il fan-out di
	// arm_busy senza servire a niente.
	//
	// MOTIVO DELLA RIMOZIONE (misurato su HW, DK Arcade DPC+): lo stallo teneva
	// RDY basso per l'INTERA durata del run ARM. Con un run che non termina il
	// 6507 restava congelato per sempre -> nessuna scrittura VSYNC -> TIA nel
	// ramo di emergenza `&v_count` -> frame ~512 righe, V = 30.70 Hz. Ma anche
	// con un run che termina il modello e' sbagliato in partenza: ne' la porta
	// cartuccia reale (top.sv:94-99) ne' Stella (CartDPCPlus.cxx:223-225,
	// Thumbulator.cxx:195 -> cycles = 0) fermano il 6507 durante il codice ARM.
	// E' lo stesso meccanismo gia' rimosso per CDF col Livello 2.
	//
	// LEZIONE DA NON PERDERE (vale per qualunque futuro segnale verso RDY):
	// `dpcp_stall_r` non era decorativo. Senza quel registro, il segnale creava
	// un percorso COMBINATORIO da arm_halted_sync (uscita del cdc_sync2 dentro
	// il subsystem) fino a RDY a top level, dove prima esisteva solo un percorso
	// REGISTRATO (la FSM S_DWAIT di cdf_bridge). Il fitter ripartizionava la
	// duplicazione di `halted` (fan-out 271 -> 252, placement FF_X42_Y19_N14 ->
	// FF_X43_Y5_N26, con PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION ON) e il path
	// critico ir->r[] dentro thumb_core scendeva da 60.60 a 57.76 MHz.
	// Chi in futuro aggiungesse un nuovo termine a RDY che nasce nel dominio
	// ARM deve registrarlo allo stesso modo.
	//------------------------------------------------------------------------

	// Flags:
	// bit 0 - direct_do in use
	// bit 1 - bitwise & direct_do and rom_do
	assign sel_flags_out = flags_out[mapper];
	assign sel_direct_do = direct_do[mapper];
	assign sel_out_en = out_en[mapper];
	assign sel_ram_rw = ram_rw[mapper];
	assign sel_ram_sel = ram_sel[mapper];
	assign sel_ram_a = ram_a[mapper];
	assign rom_a = rom_addr[mapper] & ((mapper == BANKE7 || mapper == BANK3F) ? rom_mask : {19{1'b1}});
	assign oe = out_en[mapper];

	always_comb begin
		d_out = open_bus;
		if (is_bad_game)
			d_out = bg_data;
		else if (|sel_out_en) begin
			if (sel_flags_out[0])
				d_out = sel_direct_do;
			else if (sel_flags_out[1])
				d_out = (sel_direct_do & rom_do);
			else if (sel_ram_sel) begin
				if (sel_ram_rw)
					d_out = cr_do;
			end else
				d_out = rom_do;
		end
	end

	// Since atari added no clock signal to the cart slot, for most mappers this will be the
	// primary way that they detected when to take action. The address changes typically
	// occur just before or just after phi2 on a real system. On some systems, A12 is delayed
	// in an atypical way causing this to trigger incorrectly for some games, however this
	// design does not reproduce that issue.
	always @(posedge clk) begin :reset_2600_cart
		old_ain <= a_in;
	end

	// Bank CTY is compatible with F4 minus the ARM enhanced music
	assign direct_do[BANKCTY]     = direct_do[BANKF4];
	assign flags_out[BANKCTY]     = flags_out[BANKF4];
	assign out_en[BANKCTY]        = out_en[BANKF4];
	assign ram_sel[BANKCTY]       = ram_sel[BANKF4];
	assign ram_rw[BANKCTY]        = ram_rw[BANKF4];
	assign ram_a[BANKCTY]         = ram_a[BANKF4];
	assign rom_addr[BANKCTY]      = rom_addr[BANKF4];

	// Generic CDF and DPC+ ARM execution handled via atari2600_arm_subsystem!
	assign direct_do[BANKCDF]     = arm_enable ? arm_dout : bg_data;
	assign flags_out[BANKCDF]     = 16'd1;
	assign out_en[BANKCDF]        = a_in[12] ? 8'hFF : 8'h00;
	assign ram_sel[BANKCDF]       = 0;
	assign ram_rw[BANKCDF]        = 1;
	assign ram_a[BANKCDF]         = '0;
	assign rom_addr[BANKCDF]      = arm_enable ? arm_rom_a : '0;

	// STEP 3: lo slot BANKDPCP e' RIPUNTATO sul motore DPC+ dedicato (era il
	// motore CDF via arm_dout/arm_rom_a). dpcplus_bridge ha ROM/RAM proprie in
	// M10K, quindi non serve indirizzo SDRAM: rom_addr resta a 0.
	assign direct_do[BANKDPCP]    = arm_enable ? dpcp_dout : bg_data;
	assign flags_out[BANKDPCP]    = 16'd1;
	assign out_en[BANKDPCP]       = a_in[12] ? 8'hFF : 8'h00;
	assign ram_sel[BANKDPCP]      = 0;
	assign ram_rw[BANKDPCP]       = 1;
	assign ram_a[BANKDPCP]        = '0;
	assign rom_addr[BANKDPCP]     = '0;

	assign cartram_addr = sel_ram_a;
	assign cartram_wr = sel_ram_sel && ~sel_ram_rw && ~phi1 && ~address_change;
	assign cartram_rd = sel_ram_sel &&  sel_ram_rw && ~phi1 && ~address_change;
	assign cartram_wrdata = d_in;
	assign cr_do = cartram_data;

	// Other?
	// SV   -- Spectravideo Compumate (seems useless)
	// 0840 -- Econobanking (can't find any games that use it)
	// MC   -- Megacart (doesn't seem like it works on real hardware, also no games)
	// X07  -- X07 Atariage (seems impossible, also cant find any games with it)
	// 4A50 -- 4A50 (never found a game with this)
	// FA2  -- FA2 (some kind of flash cart abstraction? Only one homebrew uses)

	mapper_none mapper_none
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK00]),
		.flags_out  (flags_out[BANK00]),
		.oe         (out_en[BANK00]),
		.ram_sel    (ram_sel[BANK00]),
		.ram_rw     (ram_rw[BANK00]),
		.ram_a      (ram_a[BANK00]),
		.rom_a      (rom_addr[BANK00])
	);

	mapper_F8 mapper_F8
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF8]),
		.flags_out  (flags_out[BANKF8]),
		.oe         (out_en[BANKF8]),
		.ram_sel    (ram_sel[BANKF8]),
		.ram_rw     (ram_rw[BANKF8]),
		.ram_a      (ram_a[BANKF8]),
		.rom_a      (rom_addr[BANKF8])
	);

	mapper_F6 mapper_F6
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF6]),
		.flags_out  (flags_out[BANKF6]),
		.oe         (out_en[BANKF6]),
		.ram_sel    (ram_sel[BANKF6]),
		.ram_rw     (ram_rw[BANKF6]),
		.ram_a      (ram_a[BANKF6]),
		.rom_a      (rom_addr[BANKF6])
	);

	mapper_FE mapper_FE
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKFE]),
		.flags_out  (flags_out[BANKFE]),
		.oe         (out_en[BANKFE]),
		.ram_sel    (ram_sel[BANKFE]),
		.ram_rw     (ram_rw[BANKFE]),
		.ram_a      (ram_a[BANKFE]),
		.rom_a      (rom_addr[BANKFE]),
		.ce         (phi1)
	);

	mapper_E0 mapper_E0
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKE0]),
		.flags_out  (flags_out[BANKE0]),
		.oe         (out_en[BANKE0]),
		.ram_sel    (ram_sel[BANKE0]),
		.ram_rw     (ram_rw[BANKE0]),
		.ram_a      (ram_a[BANKE0]),
		.rom_a      (rom_addr[BANKE0])
	);

	mapper_3F mapper_3F
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK3F]),
		.flags_out  (flags_out[BANK3F]),
		.oe         (out_en[BANK3F]),
		.ram_sel    (ram_sel[BANK3F]),
		.ram_rw     (ram_rw[BANK3F]),
		.ram_a      (ram_a[BANK3F]),
		.rom_a      (rom_addr[BANK3F])
	);

	mapper_F4 mapper_F4
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF4]),
		.flags_out  (flags_out[BANKF4]),
		.oe         (out_en[BANKF4]),
		.ram_sel    (ram_sel[BANKF4]),
		.ram_rw     (ram_rw[BANKF4]),
		.ram_a      (ram_a[BANKF4]),
		.rom_a      (rom_addr[BANKF4])
	);

	mapper_P2 mapper_P2
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKP2]),
		.flags_out  (flags_out[BANKP2]),
		.oe         (out_en[BANKP2]),
		.ram_sel    (ram_sel[BANKP2]),
		.ram_rw     (ram_rw[BANKP2]),
		.ram_a      (ram_a[BANKP2]),
		.rom_a      (rom_addr[BANKP2]),
		.ce         (ce)
	);

	mapper_FA mapper_FA
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKFA]),
		.flags_out  (flags_out[BANKFA]),
		.oe         (out_en[BANKFA]),
		.ram_sel    (ram_sel[BANKFA]),
		.ram_rw     (ram_rw[BANKFA]),
		.ram_a      (ram_a[BANKFA]),
		.rom_a      (rom_addr[BANKFA])
	);

	mapper_CV mapper_CV
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKCV]),
		.flags_out  (flags_out[BANKCV]),
		.oe         (out_en[BANKCV]),
		.ram_sel    (ram_sel[BANKCV]),
		.ram_rw     (ram_rw[BANKCV]),
		.ram_a      (ram_a[BANKCV]),
		.rom_a      (rom_addr[BANKCV])
	);

	mapper_2K mapper_2K
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK2K]),
		.flags_out  (flags_out[BANK2K]),
		.oe         (out_en[BANK2K]),
		.ram_sel    (ram_sel[BANK2K]),
		.ram_rw     (ram_rw[BANK2K]),
		.ram_a      (ram_a[BANK2K]),
		.rom_a      (rom_addr[BANK2K])
	);

	mapper_UA mapper_UA
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKUA]),
		.flags_out  (flags_out[BANKUA]),
		.oe         (out_en[BANKUA]),
		.ram_sel    (ram_sel[BANKUA]),
		.ram_rw     (ram_rw[BANKUA]),
		.ram_a      (ram_a[BANKUA]),
		.rom_a      (rom_addr[BANKUA])
	);

	mapper_E7 mapper_E7
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKE7]),
		.flags_out  (flags_out[BANKE7]),
		.oe         (out_en[BANKE7]),
		.ram_sel    (ram_sel[BANKE7]),
		.ram_rw     (ram_rw[BANKE7]),
		.ram_a      (ram_a[BANKE7]),
		.rom_a      (rom_addr[BANKE7])
	);

	mapper_F0 mapper_F0
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKF0]),
		.flags_out  (flags_out[BANKF0]),
		.oe         (out_en[BANKF0]),
		.ram_sel    (ram_sel[BANKF0]),
		.ram_rw     (ram_rw[BANKF0]),
		.ram_a      (ram_a[BANKF0]),
		.rom_a      (rom_addr[BANKF0])
	);

	mapper_32 mapper_32
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK32]),
		.flags_out  (flags_out[BANK32]),
		.oe         (out_en[BANK32]),
		.ram_sel    (ram_sel[BANK32]),
		.ram_rw     (ram_rw[BANK32]),
		.ram_a      (ram_a[BANK32]),
		.rom_a      (rom_addr[BANK32]),
		.cold_reset (mapper != BANK32)
	);

	mapper_AR mapper_AR
	(
		.clk        (clk),
		.reset      (reset || mapper != BANKAR),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKAR]),
		.flags_out  (flags_out[BANKAR]),
		.oe         (out_en[BANKAR]),
		.ram_sel    (ram_sel[BANKAR]),
		.ram_rw     (ram_rw[BANKAR]),
		.ram_a      (ram_a[BANKAR]),
		.rom_a      (rom_addr[BANKAR]),
		.ce         (ce),
		.ar_read    (ar_read),
		.rom_do     (rom_do),
		.rom_size   (rom_size),
		.audio_data (tape_audio),
		.tape_in    (tape_in),
		.fix_sc_cs  (fix_sc_cs)
	);

	mapper_WD mapper_WD
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKWD]),
		.flags_out  (flags_out[BANKWD]),
		.oe         (out_en[BANKWD]),
		.ram_sel    (ram_sel[BANKWD]),
		.ram_rw     (ram_rw[BANKWD]),
		.ram_a      (ram_a[BANKWD]),
		.rom_a      (rom_addr[BANKWD])
	);

	mapper_3E mapper_3E
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANK3E]),
		.flags_out  (flags_out[BANK3E]),
		.oe         (out_en[BANK3E]),
		.ram_sel    (ram_sel[BANK3E]),
		.ram_rw     (ram_rw[BANK3E]),
		.ram_a      (ram_a[BANK3E]),
		.rom_a      (rom_addr[BANK3E]),
		.rom_size   (rom_size)
	);

	mapper_SB mapper_SB
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKSB]),
		.flags_out  (flags_out[BANKSB]),
		.oe         (out_en[BANKSB]),
		.ram_sel    (ram_sel[BANKSB]),
		.ram_rw     (ram_rw[BANKSB]),
		.ram_a      (ram_a[BANKSB]),
		.rom_a      (rom_addr[BANKSB])
	);

	mapper_EF mapper_EF
	(
		.clk        (clk),
		.reset      (reset),
		.a_change   (address_change),
		.sc         (sc),
		.a_in       (a_in),
		.d_in       (d_in),
		.d_out      (direct_do[BANKEF]),
		.flags_out  (flags_out[BANKEF]),
		.oe         (out_en[BANKEF]),
		.ram_sel    (ram_sel[BANKEF]),
		.ram_rw     (ram_rw[BANKEF]),
		.ram_a      (ram_a[BANKEF]),
		.rom_a      (rom_addr[BANKEF])
	);

endmodule
