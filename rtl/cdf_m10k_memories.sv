// Cyclone-V M10K espliciti per ROM e Display RAM del bridge CDF.
//
// cdf_rom_m10k e' ORA true DUAL-CLOCK (Livello 1 del fix jitter, come
// harmony_m10k_tdp.sv):
//   - porta A su clk_a (clk_vid): ARM-read (fetch istruzioni + LDR literal).
//     L'ARM non scrive MAI region 0, quindi wren_a e' cablato a 0 dal chiamante.
//   - porta B su clk_b (clk_sys): engine 6502-read (Fast Fetch/program bytes)
//     E loader-write durante il download (i due sono mutuamente esclusivi nel
//     tempo: durante cart_download il 6502 e' in reset e non legge). Porta B e'
//     percio' passata da read-only a read-write.
// Regola anti-errore Intel 272006/287078/12152 applicata simmetricamente:
// tutti i _reg_b coerenti su CLOCK1, entrambi clock0/clock1 connessi.
`default_nettype none

// 4 agosto 2026 - ROM SDOPPIATA IN DUE BANCHI, per servire il FETCH a 64 bit.
//
// Perche': misurato che il 67-78% degli ingressi in S_FETCH del core Thumb
// chiede la word SUBITO SUCCESSIVA all'ultima letta (Elevator Agent 78%, e il
// fetch e' il 25% dei suoi cicli ARM). Una lettura che porta DUE word invece di
// una elimina quei fetch. Non costa Fmax - la profondita' del mux di lettura
// della cache resta 32:1, cambia solo la larghezza del dato - e non costa
// memoria: 32768x32 e 2 x 16384x32 sono gli stessi bit negli stessi M10K.
//
// NON si usa `altsyncram` a larghezza mista 64/32: in questo progetto ha gia'
// fatto fallire la sintesi (Error 272006, vedi dpcplus_bridge.sv). Si usano due
// istanze da 16384x32, banco PARI (word di indice pari) e banco DISPARI.
//
// BANCHI RUOTATI. Indirizzando i due banchi con lo stesso indice si otterrebbe
// la coppia allineata (2N, 2N+1): per una word W dispari il compagno sarebbe
// W-1, gia' eseguita, e meta' dei fetch sequenziali resterebbe scoperta.
// Indirizzando invece il banco PARI a idx+1 quando W e' dispari si ottiene
// sempre la coppia (W, W+1) qualunque sia la parita' di W, e i fetch
// sequenziali sono coperti TUTTI. Costa un incrementatore a 14 bit sul
// cammino dell'indirizzo, che e' a monte del registro di indirizzo della M10K
// e ha margine (il cammino critico e' quello di LETTURA, dal 4 ago non
// registrato).
//
// La porta B (lato 6502 + loader) resta a UNA word: si sceglie il banco con
// addr_b[0]. La scrittura del loader va al solo banco giusto; la selezione in
// lettura va RITARDATA DI UN CICLO, tanti quanti sono gli stadi di registro
// che quella porta ha in lettura: dal 7 ago 2026 c'e' il solo registro di
// indirizzo (address_reg_b su CLOCK1, outdata_reg_b UNREGISTERED), quindi il
// dato esce UN ciclo dopo l'indirizzo che lo ha scelto.
module cdf_rom_m10k #(
  parameter INIT_FILE = ""
)(
  input  wire        clk_a,        // porta A: clk_vid (ARM read)
  input  wire [14:0] addr_a,       // indirizzo di WORD
  // PASSO 15: addr_a + 1 (in parole), calcolato A MONTE da un
  // sommatore parallelo. Toglie il `+1` che stava QUI in serie al
  // sommatore dell'indirizzo del core: valeva 1,80 ns sul cammino
  // critico, ed era il secondo di due sommatori in fila.
  input  wire [14:0] addr_a_nx,
  output wire [31:0] q_a,          // word ad addr_a
  output wire [31:0] q_a_next,     // word ad addr_a+1 (per il fetch a 64 bit)
  input  wire        clk_b,        // porta B: clk_sys (engine 6502 read + loader write)
  input  wire [14:0] addr_b,
  input  wire [31:0] data_b,
  input  wire [3:0]  byteena_b,
  input  wire        wren_b,
  output wire [31:0] q_b
);
  wire [13:0] idx_a  = addr_a[14:1];
  wire [13:0] a_even = addr_a[0] ? addr_a_nx[14:1] : idx_a;  // rotazione (mux, non somma)
  wire [13:0] a_odd  = idx_a;
  wire [13:0] idx_b  = addr_b[14:1];

  wire [31:0] qe_a, qo_a, qe_b, qo_b;
  assign q_a      = addr_a[0] ? qo_a : qe_a;
  assign q_a_next = addr_a[0] ? qe_a : qo_a;

  // Selezione del banco in lettura sulla porta B, allineata alla LATENZA DELLA
  // PORTA. 7 ago 2026: la latenza e' passata da 2 cicli a 1 (outdata_reg_b
  // UNREGISTERED), quindi il ritardo di selezione passa da due stadi a uno.
  // Il bit di banco deve arrivare al mux nello STESSO ciclo in cui esce il
  // dato: con l'uscita non registrata q_b vale mem[addr_b del ciclo prima],
  // e sel_b_d1 e' esattamente addr_b[0] del ciclo prima.
  reg sel_b_d1 = 1'b0;
  always @(posedge clk_b) sel_b_d1 <= addr_b[0];
  assign q_b = sel_b_d1 ? qo_b : qe_b;

  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (32), .widthad_a(14), .numwords_a(16384), .width_byteena_a(4),
    .width_b                       (32), .widthad_b(14), .numwords_b(16384), .width_byteena_b(4),
    // 4 ago 2026: uscita porta A non registrata, stessa motivazione documentata
    // in harmony_m10k_tdp.sv. Questa e' la ROM, da cui viene ogni FETCH.
    // 7 ago 2026: anche la porta B (vedi harmony_m10k_tdp.sv per il perche').
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
  ) ram_even (
    .clock0        (clk_a),  .clock1(clk_b),
    .address_a     (a_even), .data_a(32'd0), .byteena_a(4'b1111), .wren_a(1'b0), .q_a(qe_a),
    .address_b     (idx_b),  .data_b(data_b), .byteena_b(byteena_b),
    .wren_b        (wren_b & ~addr_b[0]), .q_b(qe_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );

  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (32), .widthad_a(14), .numwords_a(16384), .width_byteena_a(4),
    .width_b                       (32), .widthad_b(14), .numwords_b(16384), .width_byteena_b(4),
    .outdata_reg_a                 ("UNREGISTERED"),
    .outdata_reg_b                 ("UNREGISTERED"),   // 7 ago 2026, come il banco pari
    .address_reg_b                 ("CLOCK1"),
    .indata_reg_b                  ("CLOCK1"),
    .wrcontrol_wraddress_reg_b     ("CLOCK1"),
    .byteena_reg_b                 ("CLOCK1"),
    .read_during_write_mode_port_a ("DONT_CARE"),
    .read_during_write_mode_port_b ("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized        ("FALSE"),
    .init_file                     (INIT_FILE)
  ) ram_odd (
    .clock0        (clk_a),  .clock1(clk_b),
    .address_a     (a_odd),  .data_a(32'd0), .byteena_a(4'b1111), .wren_a(1'b0), .q_a(qo_a),
    .address_b     (idx_b),  .data_b(data_b), .byteena_b(byteena_b),
    .wren_b        (wren_b & addr_b[0]), .q_b(qo_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule

module cdf_display_m10k (
  input  wire        clk,
  input  wire [11:0] addr_a,
  input  wire [7:0]  data_a,
  input  wire        wren_a,
  output wire [7:0]  q_a,
  input  wire [11:0] addr_b,
  output wire [7:0]  q_b
);
  altsyncram #(
    .intended_device_family        ("Cyclone V"),
    .lpm_type                      ("altsyncram"),
    .operation_mode                ("BIDIR_DUAL_PORT"),
    .ram_block_type                ("M10K"),
    .width_a                       (8), .widthad_a(12), .numwords_a(4096), .width_byteena_a(1),
    .width_b                       (8), .widthad_b(12), .numwords_b(4096), .width_byteena_b(1),
    .outdata_reg_a                 ("CLOCK0"),
    .outdata_reg_b                 ("CLOCK0"),
    .address_reg_b                 ("CLOCK0"),
    .indata_reg_b                  ("CLOCK0"),
    .wrcontrol_wraddress_reg_b     ("CLOCK0"),
    .byteena_reg_b                 ("CLOCK0"),
    .read_during_write_mode_port_a ("DONT_CARE"),
    .read_during_write_mode_port_b ("DONT_CARE"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .power_up_uninitialized        ("FALSE")
  ) ram_i (
    .clock0        (clk),
    .address_a     (addr_a), .data_a(data_a), .byteena_a(1'b1), .wren_a(wren_a), .q_a(q_a),
    .address_b     (addr_b), .data_b(8'd0),    .byteena_b(1'b1), .wren_b(1'b0),  .q_b(q_b),
    .aclr0         (1'b0),   .aclr1(1'b0),
    .addressstall_a(1'b0),   .addressstall_b(1'b0),
    .clocken0      (1'b1),   .clocken1(1'b1), .clocken2(1'b1), .clocken3(1'b1),
    .rden_a        (1'b1),   .rden_b(1'b1),
    .eccstatus     ()
  );
endmodule
`default_nettype wire
