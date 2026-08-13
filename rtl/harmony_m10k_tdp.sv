// Cyclone-V M10K true DUAL-CLOCK dual-port (Livello 1 del fix jitter).
//
// STORIA: la versione precedente era mono-clock (clock0 su entrambe le porte,
// tutti i parametri _reg_b forzati a CLOCK0). Quella scelta serviva a evitare
// gli errori Intel 272006/287078/12152, che si presentano quando si MISCELANO
// default e override sui registri di porta B: in BIDIR_DUAL_PORT il default
// Intel per i _reg_b e' CLOCK1, quindi con un solo clock connesso (clock0)
// lasciare un _reg_b a default lo aggancia a un clock1 inesistente -> errore.
// La soluzione mono-clock era forzare TUTTI i _reg_b a CLOCK0.
//
// ORA (dual-clock): la porta A va su clk_a (clk_vid, uso diretto da thumb_core
// senza handshake CDC) e la porta B su clk_b (clk_sys, uso da cdf_bridge,
// invariato). La regola anti-errore e' la stessa, applicata simmetricamente:
// TUTTI i _reg_b coerenti su CLOCK1 (nessuno a default, nessuno misto) e
// ENTRAMBI clock0/clock1 connessi. Questo e' esattamente il caso d'uso
// previsto da BIDIR_DUAL_PORT, non una forzatura.
//
// READ-DURING-WRITE mixed-port = DONT_CARE: se le due porte accedono alla
// stessa word negli intorni temporali dei rispettivi domini, il dato letto
// non e' garantito (vecchio/nuovo/misto) — NON e' corruzione fisica della
// cella.
//
// ATTENZIONE, 4 agosto 2026 - QUESTA NOTA ERA OBSOLETA E FUORVIANTE.
// Diceva che il rischio era coperto da due protezioni: lo stallo
// `arm_cpu_stall`, che avrebbe impedito al 6502 di leggere mentre l'ARM scrive,
// e il ping-pong della Display Data come rete di sicurezza definitiva.
// NESSUNA DELLE DUE E' PIU' IN VIGORE:
//   - `arm_cpu_stall` non gate piu' RDY: `top.sv` fa
//         assign RDY = tia_RDY & ~ds_wait & ~dpcp_arm_stall;
//     (Livello 2, rimozione dello stallo, gia' fatta). Resta solo lo stallo
//     specifico DPC+, che non copre il CDF/CDFJ.
//   - il ping-pong e' presente come infrastruttura ma DISATTIVO
//     (`PINGPONG_ACTIVE = 1'b0` in cdf_bridge.sv): manca la logica di switch.
// Quindi oggi la collisione read-during-write e' scoperta. Misurata su Lode
// Runner: ZERO collisioni in 15 frame, quindi non e' la causa dei guasti noti,
// ma resta un pericolo latente reale e non va piu' dato per chiuso.
`default_nettype none
// Livello 3 (ping-pong Display Data): la RAM e' allargata da 2048 a 4096 word
// (16KB) per ospitare un SECONDO buffer Display Data. AW parametrizza la
// larghezza dell'indirizzo (default 12 = 4096 word). Mappa word:
//   0..511     driver + datastream pointer/increment + C-vars basse (NON duplicati)
//   512..1535  Display Data buffer 0
//   1536..2047 C-vars/stack alte (NON duplicate)
//   2560..3583 Display Data buffer 1  (= buffer0 + 2048, offset potenza di 2)
// La selezione buffer0/buffer1 avviene nel bridge via disp_bank (bit alto).
module harmony_m10k_tdp #(
  parameter INIT_FILE = "",
  parameter AW = 12                    // address width (4096 word)
)(
  input  wire          clk_a,          // porta A: clk_vid (thumb_core)
  input  wire [AW-1:0] addr_a,
  input  wire [31:0]   data_a,
  input  wire [3:0]    byteena_a,
  input  wire          wren_a,
  output wire [31:0]   q_a,
  input  wire          clk_b,          // porta B: clk_sys (cdf_bridge engine 6502)
  input  wire [AW-1:0] addr_b,
  input  wire [31:0]   data_b,
  input  wire [3:0]    byteena_b,
  input  wire          wren_b,
  output wire [31:0]   q_b
);
  localparam NW = (1 << AW);
  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (32), .widthad_a(AW), .numwords_a(NW), .width_byteena_a(4),
    .width_b                       (32), .widthad_b(AW), .numwords_b(NW), .width_byteena_b(4),
    // Porta A registrata su CLOCK0 (clk_a), porta B su CLOCK1 (clk_b).
    //
    // 4 agosto 2026 - LATENZA DI LETTURA. Con il registro di uscita la porta A
    // ha 2 cicli di latenza e ogni lettura dell'ARM costa 3 cicli di S_MEM.
    // Misurato che fetch+memoria valgono il 65% del tempo ARM
    // (vedi prefetch-esito-negativo), quindi togliere UN ciclo a ogni lettura
    // vale piu' di qualunque riorganizzazione del fetch. Prezzo: il dato esce
    // combinatorio dalla M10K e finisce sul cammino critico verso il register
    // file, che ha poco margine. Il verdetto lo da' il fit, non la simulazione.
    // 7 agosto 2026 - ANCHE LA PORTA B SENZA REGISTRO DI USCITA.
    // Qui NON serve a guadagnare velocita': il ciclo del 6507 dura 12 clk_sys
    // qualunque cosa faccia il bridge. Serve a fare STARE il serve dentro quel
    // budget con margine: da 10 cicli su 12 (margine 2) a 7 su 12 (margine 5),
    // cosi' le attese brevi del datastream si concludono invece di essere
    // troncate dal trigger del ciclo successivo (Lode Runner).
    // ATTENZIONE: con l'uscita non registrata il dato e' valido SOLO nel ciclo
    // successivo all'indirizzo, non due. OGNI consumatore della porta B va
    // adeguato insieme - adeguarne uno solo e' garanzia di guasto (provato il
    // 6 agosto). L'elenco completo sta in cdf_bridge.sv, sopra la FSM.
    .outdata_reg_a                 ("UNREGISTERED"),
    .outdata_reg_b                 ("UNREGISTERED"),
    .address_reg_b                 ("CLOCK1"),
    .indata_reg_b                  ("CLOCK1"),
    .wrcontrol_wraddress_reg_b     ("CLOCK1"),
    .byteena_reg_b                 ("CLOCK1"),
    .read_during_write_mode_port_a ("DONT_CARE"),
    .read_during_write_mode_port_b ("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized        ("FALSE"),
    .init_file                     (INIT_FILE)
  ) ram_i (
    .clock0        (clk_a),  // porta A
    .clock1        (clk_b),  // porta B
    .address_a     (addr_a), .data_a(data_a), .byteena_a(byteena_a), .wren_a(wren_a), .q_a(q_a),
    .address_b     (addr_b), .data_b(data_b), .byteena_b(byteena_b), .wren_b(wren_b), .q_b(q_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule
`default_nettype wire
