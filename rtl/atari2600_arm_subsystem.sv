//============================================================================
// Module: atari2600_arm_subsystem.sv
// Description: ARM Thumb + CDF bridge subsystem for CDF/CDFJ/CDFJ+ carts.
//              Wires the 6502 bus into cdf_bridge, runs thumb_core once per
//              CALLFN (Stella/Thumbulator semantics: the ARM is halted until
//              the 6502 writes 254/255 to $1FF3, each run re-initializes the
//              registers, and the run ends when the driver main() returns),
//              and services the driver's music callbacks via the bridge.
//============================================================================

`default_nettype none

module atari2600_arm_subsystem (
    input  wire        clk,            // System master clock (~14.3 MHz) - 6502/TIA/cdf_bridge domain
    input  wire        clk_vid,        // Phase B: ARM Thumb core domain (~57.3 MHz, PLL 4x clk_sys)
    input  wire        reset,          // System warm/cold reset
    input  wire        arm_enable,     // Master enable switch (OSD)
    input  wire [4:0]  mapper,         // Autodetected mapper ID (BANKCDF=22, BANKDPCP=20)
    input  wire [1:0]  cdf_family,     // From detect2600: 00=CDF0/1 01=CDFJ 10=CDFJ+

    // 6507 CPU bus ($1000-$1FFF cartridge window)
    input  wire [12:0] a_in,           // 6502 address in cartridge window
    input  wire [7:0]  d_in,           // 6502 data bus input (write data on writes)
    input  wire        rwn,            // 6502 R/W_n: 1 = read cycle, 0 = write cycle
    output wire [7:0]  d_out,          // 6502 data bus output

    // HPS cartridge download (preloads ROM + Harmony RAM driver image)
    input  wire        cart_download,
    input  wire [18:0] ioctl_addr,
    input  wire [7:0]  ioctl_dout,
    input  wire        ioctl_wr,

    // SDRAM interconnect (unused: ROM/RAM are internal M10K; keep tied off)
    input  wire [7:0]  rom_do,
    input  wire [18:0] rom_size,
    output wire [18:0] rom_a,
    output wire        rom_read,

    // Livello 3 (ping-pong Display Data): fase Kernel/VBlank (bit VBLANK software
    // dalla TIA, clk_sys) per la logica di switch A/B/C.
    input  wire        vblank_sw,

    // FASE9 (attesa selettiva): richiesta di wait-state sul 6507, alzata dal
    // bridge SOLO durante un serve datastream che cadrebbe mentre un run ARM e'
    // ancora attivo nel Kernel. Si combina in top.sv con RDY della TIA
    // (RDY = tia_RDY & ~ds_wait). Gate a LIVELLO, nessun latch: cade da solo.
    output wire        ds_wait,

    // Telemetry
    output wire        busy,           // ARM run in progress
    output wire        done,           // ARM halted (idle)
    output wire [7:0]  audio_mix_out,  // unused (audio flows via AMPLITUDE reads)
    // STEP B: bus ARM esposto verso dpcplus_bridge (istanziato in cart2600).
    // Il thumb_core resta uno solo: qui esce il lato "master" e rientrano i
    // ritorni del ramo DPC+, muxati sotto con quelli di cdf_bridge.
    output wire        arm_rst_vid,
    output wire [31:0] arm_bus_addr,
    output wire [31:0] arm_bus_wdata,
    output wire [3:0]  arm_bus_be,
    output wire        arm_bus_we,
    output wire        arm_bus_req_dpcp,
    // Indirizzo anticipato verso il bridge DPC+ (stadio 2).
    output wire [31:0] arm_bus_addr_pre,
    output wire        arm_bus_req_pre,
    input  wire [31:0] arm_bus_rdata_dpcp,
    input  wire        arm_bus_ack_dpcp,
    // STEP C: CALLFUNCTION dal bridge DPC+ (avvia il thumb_core condiviso)
    input  wire        dpcp_callfn,
    input  wire [7:0]  dpcp_callfn_val,
    output wire [31:0] dbg_pc,
    output wire [31:0] dbg_r0,
    // 6 ago 2026: sonde del ping-pong (swap_pending / arm_halt_rising /
    // vblank_start). Esistono per essere lette dal testbench; portarle fuori
    // serve a far vedere a Quartus che sono usate, senza doverle togliere.
    output wire [2:0]  dbg_pp
);

    //========================================================================
    // Activation & reset
    //========================================================================
    // STEP 3: ristretto al SOLO BANKCDF (22). DPC+ (20) ha ora il proprio
    // motore dedicato (dpcplus_bridge in cart2600.sv) e non usa piu' questo
    // subsystem. Conseguenza voluta del PASSO 1: con DPC+ il subsystem resta
    // in core_reset, quindi CALLFUNCTION non esegue codice ARM (i fetcher
    // funzionano, l'ARM arrivera' nel PASSO 2).
    wire is_cdf_mapper = (mapper == 5'd22);
    wire subsys_active = arm_enable & is_cdf_mapper;
    wire core_reset    = reset | (~subsys_active) | cart_download;

    wire is_dpcp = (mapper == 5'd20);

    //------------------------------------------------------------------------
    // STEP C: attivazione del CORE ARM separata da quella del BRIDGE CDF.
    //
    //   subsys_active / core_reset -> restano SOLO per cdf_bridge e per tutto
    //     cio' che e' specifico del motore CDF. Semantica INVARIATA.
    //   arm_core_active / arm_core_reset -> nuovi, per il solo thumb_core.
    //     Valgono per BANKCDF (come prima) E per BANKDPCP (nuovo).
    //
    // EQUIVALENZA CON BANKCDF (dimostrazione):
    //   is_dpcp = (mapper==5'd20) e is_cdf_mapper = (mapper==5'd22) sono
    //   uguaglianze strette su valori DISTINTI dello stesso enum, quindi
    //   mutuamente esclusive. Con mapper==BANKCDF: is_dpcp=0, dunque
    //     arm_core_active = arm_enable & (is_cdf_mapper | 0) = subsys_active
    //     arm_core_reset  = reset | ~subsys_active | cart_download = core_reset
    //   cioe' le due espressioni COLLASSANO in quelle preesistenti, termine
    //   per termine. Nessuna regressione possibile sui titoli CDF.
    //------------------------------------------------------------------------
    wire arm_core_active = arm_enable & (is_cdf_mapper | is_dpcp);
    wire arm_core_reset  = reset | (~arm_core_active) | cart_download;
    wire [1:0]  family_sel = is_dpcp ? 2'b00 : cdf_family;
    wire is_plus_fam = (family_sel == 2'b10);

    //========================================================================
    // Download-time scans (Stella CartridgeCDF::setupVersion / constructor):
    //  - CDFJ+ C stack/base from image dwords $17F4/$17F8
    //  - CDFJ+ LDX#/LDY# fast-fetch enables (driver dwords 0x135200A2/A0)
    //  - CDFJ+ fast-fetch offset cell ((dword & 0xFFFFFF00) == 0xE2422000)
    //========================================================================
    reg [23:0] dw3;              // last three download bytes (little endian)
    reg [31:0] cstack_img, cbase_img;
    reg        ldx_seen, ldy_seen;
    reg [15:0] ff_offset_r;
    wire [31:0] dw_full = {ioctl_dout, dw3};

    always @(posedge clk) begin
        if (cart_download && ioctl_wr) begin
            // 2 ago 2026 - ERA `{ioctl_dout, dw3[23:16]}`: 8+8=16 bit assegnati
            // a un registro da 24, quindi lo scorrimento perdeva un byte a ogni
            // passo e dw3 valeva sempre {8'h00, byte_corrente, 8'h00}. Effetto
            // su Gorf Arcade (CDFJ+): cbase_img=0 -> l'ARM partiva da indirizzo
            // 0 ed eseguiva la tabella vettori ARM come Thumb (419430 opcode
            // ignoti su 209715 istruzioni), cstack_img=0x40000000 -> stack al
            // fondo della RAM. Servono 24 bit: {byte, dw3[23:8]}.
            dw3 <= {ioctl_dout, dw3[23:8]};
            if (ioctl_addr[1:0] == 2'b11 && ioctl_addr < 19'h800) begin
                if (dw_full == 32'h135200A2) ldx_seen <= 1'b1;
                if (dw_full == 32'h135200A0) ldy_seen <= 1'b1;
                if ((dw_full & 32'hFFFFFF00) == 32'hE2422000)
                    ff_offset_r <= {ioctl_addr[15:2], 2'b00};
            end
            if (ioctl_addr == 19'h17F7) cstack_img <= dw_full;
            if (ioctl_addr == 19'h17FB) cbase_img  <= dw_full;
        end
        if (reset && !cart_download) begin
            // keep scan results across warm reset; clear only on new download
        end
        if (cart_download && ioctl_wr && ioctl_addr == 19'd0) begin
            ldx_seen <= 1'b0; ldy_seen <= 1'b0; ff_offset_r <= 16'd0;
            cstack_img <= 32'h40001FFC; cbase_img <= 32'h00000800;
        end
    end

    //========================================================================
    // ARM start vectors (Stella: cBase/cStart/cStack per subtype)
    //   CDF/CDF1/CDFJ : cBase 0x800, cStart 0x808, cStack 0x40001FFC
    //   DPC+          : cBase 0xC00, cStart 0xC08, cStack 0x40001FFC
    //   CDFJ+         : cBase = img[$17F8] & ~1, cStart = cBase,
    //                   cStack = img[$17F4]
    //========================================================================
    wire [31:0] arm_start_pc = is_dpcp     ? 32'h00000c08 :
                               is_plus_fam ? (cbase_img & 32'hFFFFFFFE) :
                                             32'h00000808;
    wire [31:0] arm_start_sp = is_dpcp     ? 32'h40001ffc :
                               is_plus_fam ? cstack_img :
                                             32'h40001ffc;
    wire [31:0] arm_start_lr = is_dpcp     ? 32'h00000c00 :
                               is_plus_fam ? (cbase_img & 32'hFFFFFFFE) :
                                             32'h00000800;

    //========================================================================
    // Interconnect
    //========================================================================
    // Livello 1: il bus ARM (bus_addr/wdata/be/we/sz/req/ack/rdata) e' ora un
    // collegamento DIRETTO thumb_core <-> cdf_bridge, tutto su clk_vid, senza
    // CDC. La callback musicale (cb_*) resta un handshake CDC verso lo stato
    // musicale del bridge su clk_sys: cb_req_raw/cb_ack_raw sono i suoi lati
    // "raw" pre-resync (invariati).
    wire [31:0] bus_addr, bus_wdata, bus_rdata;
    // STADIO 2: indirizzo efficace anticipato di un ciclo, dal core al bridge
    wire [31:0] bus_addr_pre;
    wire [31:0] bus_addr_pre_nx;   // PASSO 15
    wire        bus_req_pre;
    wire [3:0]  bus_be;
    wire        bus_we;
    wire [1:0]  bus_sz;
    wire        bus_req, bus_ack;
    // STEP B: lati di ritorno separati dei due bridge. Il thumb_core vede
    // bus_rdata/bus_ack muxati per mapper (vedi in fondo al file).
    wire [31:0] bus_rdata_cdf, bus_rdata_dpcp;
    wire        bus_ack_cdf,   bus_ack_dpcp;
    // FETCH a 64 bit: solo il bridge CDF ha la ROM sdoppiata e sa restituire la
    // word successiva. Col DPC+ il segnale di validita' resta 0 e il core si
    // comporta esattamente come prima (riempie una sola entry di cache).
    wire [31:0] bus_rdata_next_cdf;
    wire        bus_rdata_next_ok_cdf;
    wire [31:0] bus_rdata_next    = bus_rdata_next_cdf;
    wire        bus_rdata_next_ok = is_cdf_mapper & bus_rdata_next_ok_cdf;
    wire        cb_req_raw;
    wire        cb_ack_raw;

    wire [7:0]  bridge_dout;
    wire        callfn;
    wire [7:0]  callfn_val;
    wire [1:0]  cb_id;
    wire [31:0] cb_v1, cb_v2, cb_ret;
    wire        arm_halted;

    // NOTE: a_in[12] is the $1000-window select; the padded address must be
    // $1xxx exactly when a_in[12]=1 (the old {3'b001,a_in} produced $3xxx and
    // the bridge chip-select never fired).
    wire [15:0] m6502_addr_full = {3'b000, a_in[12:0]};

    // callback trap addresses for this family
    wire [31:0] cb0, cb1, cb2, cb3;
    wire [15:0] p_nc0, p_nc1, p_nc2, p_nc3;
    wire [7:0]  p_nc4, p_nc5; wire p_nc6; wire [2:0] p_nc7;
    cdf_family_params params_cb (
        .family_sel(family_sel),
        .ds_base(p_nc0), .ds_inc_base(p_nc1), .wf_base(p_nc2), .prog_off(p_nc3),
        .amp_stream(p_nc4), .jump_mask(p_nc5), .is_plus(p_nc6), .start_bank(p_nc7),
        .cb0(cb0), .cb1(cb1), .cb2(cb2), .cb3(cb3)
    );

    // Start one ARM run per CALLFN 254/255 (Stella callFunction). In Stella
    // the ARM emulator runs synchronously inside callFunction() - from the
    // 6507's point of view it takes zero cycles, so a second CALLFN can
    // never arrive while a previous run is still "in progress". On real
    // hardware (and in this softcore) an ARM run takes hundreds to
    // thousands of clk cycles, so a CALLFN arriving while thumb_core is
    // still busy must be latched and honored as soon as it halts, not
    // dropped. A plain one-cycle pulse (the original implementation) was
    // silently lost whenever the 6502 issued back-to-back CALLFNs faster
    // than the ARM could finish - fine on a static title screen (few CALLFN/
    // frame, ARM always idle in time) but drops driver calls once gameplay
    // raises the CALLFN rate, which reads as the game hanging after the
    // title screen.
    //
    // Phase B CDC: thumb_core now runs on clk_vid (~57.3 MHz, PLL 4x
    // clk_sys) while cdf_bridge/callfn stay on clk (~14.3 MHz clk_sys).
    // clk_sys and clk_vid are independent altera_pll outputs with no
    // guaranteed phase relationship (see Atari2600.sdc) - treated as fully
    // asynchronous. The arm_start_pending latch itself must therefore live
    // in the clk_vid domain (it ANDs with arm_halted, which is native to
    // thumb_core/clk_vid), and the 1-clk `callfn` pulse (clk_sys domain)
    // must cross via a toggle + 2-flop resync + edge-detect scheme so it is
    // never missed even if it arrives close to a clk_vid edge (a plain
    // 2-flop level sync would lose a pulse shorter than one clk_vid period
    // relative to clk_vid, which a single clk_sys-wide pulse can be at this
    // clock ratio).
    // STEP C: la CALLFN che avvia il core arriva dal bridge ATTIVO. Con
    // BANKCDF e' callfn/callfn_val di cdf_bridge (invariato); con BANKDPCP
    // e' quella di dpcplus_bridge, entrata dalla porta dpcp_callfn.
    // is_cdf_mapper e is_dpcp sono mutuamente esclusivi (enum distinti),
    // quindi con CDF questo mux degenera nei segnali preesistenti.
    wire       arm_callfn     = is_dpcp ? dpcp_callfn     : callfn;
    wire [7:0] arm_callfn_val = is_dpcp ? dpcp_callfn_val : callfn_val;

    reg callfn_toggle;
    always @(posedge clk) begin
        if (arm_core_reset) callfn_toggle <= 1'b0;
        else if (arm_callfn && (arm_callfn_val == 8'd254 || arm_callfn_val == 8'd255))
            callfn_toggle <= ~callfn_toggle;
    end

    wire core_reset_vid;
    cdc_sync2 u_sync_reset_vid (.dst_clk(clk_vid), .d(arm_core_reset), .q(core_reset_vid));

    wire callfn_toggle_vid;
    cdc_sync2 u_sync_callfn_vid (.dst_clk(clk_vid), .d(callfn_toggle), .q(callfn_toggle_vid));
    reg callfn_toggle_vid_d;
    wire callfn_pulse_vid = callfn_toggle_vid ^ callfn_toggle_vid_d;

    reg arm_start_pending;
    wire arm_start = arm_start_pending & arm_halted;
    always @(posedge clk_vid) begin
        callfn_toggle_vid_d <= callfn_toggle_vid;
        if (core_reset_vid) arm_start_pending <= 1'b0;
        else begin
            if (callfn_pulse_vid)
                arm_start_pending <= 1'b1;
            else if (arm_start)
                arm_start_pending <= 1'b0;
        end
    end

    // halted (clk_vid, native to thumb_core) resynchronized into clk_sys for
    // busy/done telemetry and for cart2600's arm_cpu_stall/RDY gating, which
    // must observe a clean single-bit level - a 2-flop sync is sufficient
    // (no multi-bit tearing risk on a single bit).
    wire arm_halted_sync;
    cdc_sync2 u_sync_halted_sys (.dst_clk(clk), .d(arm_halted), .q(arm_halted_sync));

    //========================================================================
    // Livello 3 - ping-pong Display Data: INFRASTRUTTURA IMPLEMENTATA MA
    // DISATTIVA (disp_bank forzato a 0). Vedi FASE8_livello3_pingpong.md.
    //
    // L'infrastruttura del ping-pong e' completa e verificata come innocua
    // (RAM allargata a 4096 word, remap_disp, doppio buffer Display Data): con
    // disp_bank=0 costante il comportamento e' IDENTICO al Livello 2 (frame NTSC
    // 19912, callfn==arm_runs). Cio' che resta IRRISOLTO e' la LOGICA DI SWITCH:
    // il design FASE3b assumeva che i run ARM finissero lasciando un istante di
    // "ARM fermo in VBlank" in cui pubblicare il buffer back. La simulazione ha
    // dimostrato (con evidenza) che, rimosso lo stallo, i run ARM di Draconian
    // sono quasi-continui e finiscono SEMPRE nel Kernel, MAI in VBlank -> non
    // esiste un momento affidabile per swappare senza strappare il buffer che
    // l'ARM sta scrivendo. Swappare a vblank_start (incondizionato) mantiene la
    // coerenza logica (callfn==arm_runs MATCH) ma introduce un frame di latenza
    // e un rischio di strappo da validare su HW. La scelta della logica di
    // switch corretta richiede una fase di design dedicata (vedi documento).
    //
    // Finche' quella logica non e' definita, disp_bank resta 0: entrambe le
    // porte lavorano sul buffer 0, il ping-pong e' trasparente, il comportamento
    // e' esattamente quello del Livello 2 (nessuna regressione).
    reg disp_bank;
    reg swap_pending;
    reg arm_halted_sync_d, vblank_sw_d;
    wire arm_halt_rising = arm_halted_sync & ~arm_halted_sync_d; // fine run ARM
    wire vblank_start     = vblank_sw & ~vblank_sw_d;            // inizio finestra morta
    // 6 ago 2026: questi tre segnali RESTANO, per richiesta esplicita del PM
    // ("non togliere nessuno strumento utile di debug"). Quartus li segnalava
    // come "assegnati e mai letti" (10036) perche' l'unico lettore e' il
    // TESTBENCH, che il tool non analizza: sono le sonde della logica A/B/C del
    // ping-pong. Oggi riportano zeri, perche' il ping-pong non e' implementato
    // e `disp_bank` resta 0 - ma servono nel momento in cui lo si fara'.
    // Il warning si chiude portandoli su un'uscita di debug (vedi `dbg_pp`
    // sotto), stesso rimedio usato in cdf_bridge.sv per `lda_valid`.
    assign dbg_pp = {swap_pending, arm_halt_rising, vblank_start};
    always @(posedge clk) begin
        arm_halted_sync_d <= arm_halted_sync;
        vblank_sw_d       <= vblank_sw;
        if (core_reset) begin
            disp_bank <= 1'b0; swap_pending <= 1'b0;
        end else begin
            // DISATTIVO: disp_bank resta 0. La logica A/B/C (swap_pending su
            // arm_halt_rising, consumo al momento sicuro) e' pronta ma non
            // pilota disp_bank finche' non e' risolto il problema del "quando
            // swappare con run continui". arm_halt_rising/vblank_start restano
            // calcolati per non perdere il lavoro gia' fatto.
            disp_bank    <= 1'b0;
            swap_pending <= 1'b0;
        end
    end

    //========================================================================
    // FASE9 - ATTESA SELETTIVA sul serve datastream (ATTIVA).
    // Vedi docs/atari2600_arm_integration/FASE9_attesa_selettiva_datastream.md
    //
    // Problema: rimosso lo stallo globale (Livello 2), un run ARM puo' sconfinare
    // nel Kernel. Se in quella finestra il 6507 legge un datastream, legge il
    // puntatore PRIMA che l'ARM lo abbia aggiornato (lost update / race).
    //
    // Soluzione: posporre SOLO quell'accesso (non stallare l'intero 6507/TIA).
    // ds_wait_arm e' la condizione a LIVELLO "siamo nel Kernel e un run ARM e'
    // ancora attivo". La terza condizione ("si sta servendo un datastream")
    // vive nel bridge, che combina ds_wait_arm con lo stato della FSM e produce
    // ds_wait. NESSUN LATCH: ds_wait_arm cade da solo appena vblank_sw torna 1
    // (fine Kernel) oppure l'ARM si ferma (arm_halted_sync=1). Non c'e' un
    // buffer da pubblicare (a differenza del ping-pong), si pospone solo un
    // accesso: il livello basta ed e' auto-stabilizzante (retroazione negativa).
    //
    // Riusa arm_halted_sync (clk_sys) e vblank_sw/vblank_sw_d gia' presenti nel
    // blocco ping-pong sopra. kernel_start/arm_running sono i soli termini
    // combinatori nuovi. Nessun CDC nuovo.
    //========================================================================
    // NB: si usa il LIVELLO ~vblank_sw, non il fronte kernel_start. La FASE 1 ha
    // misurato che i run ARM entrano nel Kernel un attimo DOPO il suo inizio
    // (overrun_detect sul fronte = 0, ma 1117 cicli clk_vid di sconfinamento
    // reale): un gate a fronte non li vedrebbe.
    wire arm_running  = ~arm_halted_sync;           // run ARM ancora in corso
    wire ds_wait_arm  = ~vblank_sw & arm_running;   // gate a livello (no latch)

    // Livello 1: il bus ARM (thumb_core) NON attraversa piu' alcun CDC. Sia
    // thumb_core sia l'adapter bus di cdf_bridge girano su clk_vid, e le porte
    // A delle M10K (ROM/Harmony) sono fisicamente sul dominio clk_vid. Il
    // vecchio handshake toggle a 4 fasi (shim_addr/wdata/be/we/sz, req_toggle/
    // ack_toggle, breq_active/breq_rdata_lat) e' rimosso: bus_addr/wdata/be/we/
    // sz/req di thumb_core vanno diretti al bridge, e bus_ack/bus_rdata del
    // bridge tornano diretti a thumb_core, tutto nello stesso dominio clk_vid.
    // La sequenza arm_p1/bus_ack del bridge (ora su clk_vid) e' identica a
    // quella che thumb_core gia' vedeva, quindi la golden trace e' preservata.
    // Il problema di "level-sync che non distingue due richieste back-to-back"
    // che aveva imposto il toggle-handshake NON esiste piu': senza CDC non c'e'
    // alcun sincronizzatore di livello da confondere.
    //
    // NB: restano su CDC (invariati) i SOLI segnali di controllo/stato
    // (core_reset, callfn, halted) e l'handshake della callback musicale, che
    // NON sono accessi a memoria - vedi sotto.

    // Music-callback trap handshake CDC (same toggle-based 4-phase pattern;
    // cb_req is a plain level from thumb_core with no back-to-back-reissue
    // behavior like bus_req, but the toggle scheme is applied uniformly for
    // the same robustness guarantee and to keep cb_id/cb_v1/cb_v2 latched
    // stable across the boundary rather than relying on driver timing).
    reg [1:0]  shim_cb_id; reg [31:0] shim_cb_v1, shim_cb_v2;
    reg        cb_req_toggle;
    reg        cb_req_raw_d;
    wire       new_cb_req = cb_req_raw && !cb_req_raw_d;
    always @(posedge clk_vid) begin
        cb_req_raw_d <= cb_req_raw;
        if (core_reset_vid) cb_req_toggle <= 1'b0;
        else if (new_cb_req) begin
            shim_cb_id <= cb_id; shim_cb_v1 <= cb_v1; shim_cb_v2 <= cb_v2;
            cb_req_toggle <= ~cb_req_toggle;
        end
    end

    wire cb_req_toggle_sys;
    cdc_sync2 u_sync_cbreqtog_sys (.dst_clk(clk), .d(cb_req_toggle), .q(cb_req_toggle_sys));
    reg cb_req_toggle_sys_d;
    wire cb_req_edge_sys = cb_req_toggle_sys ^ cb_req_toggle_sys_d;

    reg        cbreq_active;
    reg [31:0] cbreq_ret_lat;
    reg        cb_ack_toggle;
    always @(posedge clk) begin
        cb_req_toggle_sys_d <= cb_req_toggle_sys;
        if (core_reset) begin
            cbreq_active <= 1'b0; cb_ack_toggle <= 1'b0;
        end else begin
            if (cb_req_edge_sys && !cbreq_active) cbreq_active <= 1'b1;
            if (cbreq_active && cb_ack_raw) begin
                cbreq_active  <= 1'b0;
                cbreq_ret_lat <= cb_ret;
                cb_ack_toggle <= ~cb_ack_toggle;
            end
        end
    end
    wire cb_req_sys = cbreq_active;

    wire cb_ack_toggle_vid;
    cdc_sync2 u_sync_cbacktog_vid (.dst_clk(clk_vid), .d(cb_ack_toggle), .q(cb_ack_toggle_vid));
    reg cb_ack_toggle_vid_d;
    always @(posedge clk_vid) cb_ack_toggle_vid_d <= cb_ack_toggle_vid;
    wire cb_ack_vid = cb_ack_toggle_vid ^ cb_ack_toggle_vid_d;
    wire [31:0] cb_ret_vid = cbreq_ret_lat;

    //========================================================================
    // CDF bridge
    //========================================================================
    cdf_bridge #(
        .ROM_DEPTH (32768),
        .RAM_DEPTH (32768)
    ) u_cdf_bridge (
        .clk          (clk),
        .clk_vid      (clk_vid),
        .rst          (core_reset),
        .rst_vid      (core_reset_vid),
        .m6502_addr   (m6502_addr_full),
        .m6502_din    (d_in),
        .m6502_rwn    (rwn),
        .m6502_dout   (bridge_dout),
        // bus ARM diretto (clk_vid), nessun CDC
        .bus_addr     (bus_addr),
        .bus_addr_pre (bus_addr_pre),
        .bus_addr_pre_nx (bus_addr_pre_nx),
        .bus_req_pre  (bus_req_pre),
        .bus_wdata    (bus_wdata),
        .bus_rdata    (bus_rdata_cdf),
        .bus_be       (bus_be),
        .bus_we       (bus_we),
        .bus_sz       (bus_sz),
        .bus_req      (bus_req & subsys_active),
        .bus_ack      (bus_ack_cdf),
        .bus_rdata_next    (bus_rdata_next_cdf),
        .bus_rdata_next_ok (bus_rdata_next_ok_cdf),
        .callfn       (callfn),
        .callfn_val   (callfn_val),
        .cb_req       (cb_req_sys),
        .cb_id        (shim_cb_id),
        .cb_v1        (shim_cb_v1),
        .cb_v2        (shim_cb_v2),
        .cb_ret       (cb_ret),
        .cb_ack       (cb_ack_raw),
        .family_sel   (family_sel),
        .ldx_en       (is_plus_fam & ldx_seen),
        .ldy_en       (is_plus_fam & ldy_seen),
        .ff_offset    (is_plus_fam ? ff_offset_r : 16'd0),
        .disp_bank    (disp_bank),
        .ds_wait_arm  (ds_wait_arm),   // FASE9: Kernel & run ARM attivo (livello)
        .arm_run_now  (arm_running & vblank_sw), // PASSO ARSP: solo FUORI dal disegno
        .ds_wait      (ds_wait),       // FASE9: wait-state sul serve datastream

        .rom_load_we  (cart_download & ioctl_wr & (ioctl_addr < 19'd131072)),
        .rom_load_addr(ioctl_addr[16:0]),
        .rom_bytes    (rom_size),
        .rom_load_data(ioctl_dout),
        .dbg_bank      (),
        .dbg_mode      (),
        .dbg_lda_valid (),
        .dbg_lda_addr  (),
        .dbg_ff_val    ()
    );

    //========================================================================
    // ARM Thumb core
    //========================================================================
    thumb_core u_arm_core (
        .clk      (clk_vid),
        .rst      (core_reset_vid),
        .start    (arm_start),
        .start_pc (arm_start_pc),
        .start_sp (arm_start_sp),
        .start_lr (arm_start_lr),
        .halted   (arm_halted),
        // I quattro trap di callback musicale esistono SOLO nel driver CDF.
        // Stella, per DPC+, non ne installa nessuno (Thumbulator.cxx:1780,
        // "case ConfigureFor::DPCplus: // no 32-bit subroutines in DPC+"):
        // un BX verso indirizzo pari termina il run e basta. Qui gli indirizzi
        // CDF (0x756..0x762) restavano attivi anche con BANKDPCP: se il codice
        // ARM di una cartuccia DPC+ passasse per quelle celle il core
        // entrerebbe in un handshake di callback inesistente. Con DPC+ si
        // presentano quattro indirizzi impossibili (dispari + fuori ROM), che
        // non possono corrispondere a `pc+2` di alcun BX.
        .cb_addr0 (is_dpcp ? 32'hFFFFFFF1 : cb0),
        .cb_addr1 (is_dpcp ? 32'hFFFFFFF3 : cb1),
        .cb_addr2 (is_dpcp ? 32'hFFFFFFF5 : cb2),
        .cb_addr3 (is_dpcp ? 32'hFFFFFFF7 : cb3),
        .cb_req   (cb_req_raw),
        .cb_id    (cb_id),
        .cb_v1    (cb_v1),
        .cb_v2    (cb_v2),
        .cb_ack   (cb_ack_vid),
        .cb_ret   (cb_ret_vid),
        .bus_addr (bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_be   (bus_be),
        .bus_we   (bus_we),
        .bus_sz   (bus_sz),
        .bus_req  (bus_req),
        .bus_ack  (bus_ack),
        .bus_rdata_next    (bus_rdata_next),
        .bus_rdata_next_ok (bus_rdata_next_ok),
        .bus_addr_pre (bus_addr_pre),
        .bus_addr_pre_nx (bus_addr_pre_nx),
        .bus_req_pre  (bus_req_pre),
        .dbg_pc   (dbg_pc),
        .dbg_r0   (dbg_r0)
    );

    //========================================================================
    // Outputs
    //========================================================================
    //========================================================================
    // STEP B - MUX DEL BUS ARM fra cdf_bridge e dpcplus_bridge.
    //
    // Selezione ancorata al mapper: is_cdf_mapper (== BANKCDF, invariato dallo
    // STEP 3) sceglie il ramo CDF; is_dpcp (== BANKDPCP) il ramo DPC+. I due
    // sono valori distinti dello stesso enum, quindi mutuamente esclusivi per
    // costruzione: non esiste stato in cui il thumb_core veda i ritorni del
    // bridge sbagliato.
    //
    // Il ramo CDF e' bit-identico a prima: con is_cdf_mapper=1 il mux degenera
    // in bus_rdata_cdf/bus_ack_cdf, esattamente i segnali che cdf_bridge
    // pilotava direttamente. Nessuna modifica alla sua semantica.
    //
    // NB STEP B: il thumb_core e' ancora tenuto in reset per DPC+ (core_reset
    // dipende da subsys_active = arm_enable & is_cdf_mapper, non toccato qui:
    // e' STEP C). Con una cartuccia DPC+ bus_req resta quindi 0 e il ramo
    // DPC+ del mux non si attiva mai: il percorso e' costruito ma inerte.
    //========================================================================
    assign bus_rdata = is_cdf_mapper ? bus_rdata_cdf :
                       is_dpcp       ? bus_rdata_dpcp : 32'd0;
    assign bus_ack   = is_cdf_mapper ? bus_ack_cdf :
                       is_dpcp       ? bus_ack_dpcp : 1'b0;

    // lato master esportato verso dpcplus_bridge
    assign arm_rst_vid      = core_reset_vid;
    assign arm_bus_addr     = bus_addr;
    assign arm_bus_addr_pre = bus_addr_pre;
    // `is_dpcp`: con CDF il bridge DPC+ e' in reset e non deve vedere
    // nessun annuncio, esattamente come per `arm_bus_req_dpcp`.
    assign arm_bus_req_pre  = bus_req_pre & arm_enable & is_dpcp;
    assign arm_bus_wdata    = bus_wdata;
    assign arm_bus_be       = bus_be;
    assign arm_bus_we       = bus_we;
    assign arm_bus_req_dpcp = bus_req & arm_enable & is_dpcp;
    assign bus_rdata_dpcp   = arm_bus_rdata_dpcp;
    assign bus_ack_dpcp     = arm_bus_ack_dpcp;

    assign d_out = subsys_active ? bridge_dout : 8'h00;
    // busy/done are clk_sys-domain telemetry (OSD/debug) - use the
    // synchronized halted level, not the raw clk_vid-domain signal.
    assign busy  = ~arm_halted_sync;
    assign done  = arm_halted_sync;
    assign audio_mix_out = 8'd0;
    // The ARM ROM/RAM live in internal M10K; never drive the shared SDRAM bus.
    assign rom_a = 19'd0;
    assign rom_read = 1'b0;

endmodule
