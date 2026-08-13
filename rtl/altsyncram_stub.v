
module altsyncram (
    input wire [byteena_width_a-1:0] byteena_a,
    input wire [byteena_width_b-1:0] byteena_b,
    input wire clock0,
    input wire clock1,
    input wire clocken0, clocken1, clocken2, clocken3,
    input wire aclr0, aclr1,
    input wire addressstall_a, addressstall_b,
    input wire rden_a, rden_b,
    input wire [widthad_a-1:0] address_a,
    input wire [widthad_b-1:0] address_b,
    input wire [width_a-1:0] data_a,
    input wire [width_b-1:0] data_b,
    input wire wren_a,
    input wire wren_b,
    output reg [width_a-1:0] q_a,
    output reg [width_b-1:0] q_b,
    output wire [31:0] eccstatus
);
    parameter byteena_width_a = 1;
    parameter byteena_width_b = 1;
    parameter widthad_a = 11;
    parameter widthad_b = 11;
    parameter width_a = 32;
    parameter width_b = 32;
    parameter lpm_type = "altsyncram";
    parameter init_file = "UNUSED";
    parameter operation_mode = "BIDIR_DUAL_PORT";

    assign eccstatus = 32'd0;
    reg [width_a-1:0] ram [0:(2**widthad_a)-1];

    // Livello 1: modello dual-clock. La porta A e' sempre su clock0; la porta B
    // usa clock1 se address_reg_b == "CLOCK1" (configurazione dual-clock nuova),
    // altrimenti clock0 (retrocompat con eventuali istanze mono-clock).
    // Registrazione address + outdata (2-cycle read latency) come le M10K reali:
    // qui modellato con un solo stadio registrato sul dato letto - sufficiente
    // per la verifica funzionale del comportamento mixed-port (write-A/read-B),
    // che e' l'obiettivo del testbench di concorrenza; la latenza esatta a 2
    // stadi e' gestita dall'adapter arm_p1 nel bridge, non qui.
    parameter address_reg_b = "CLOCK0";
    wire clock_b = (address_reg_b == "CLOCK1") ? clock1 : clock0;

    always @(posedge clock0) begin
        if (wren_a) ram[address_a] <= data_a;
        q_a <= ram[address_a];
    end

    always @(posedge clock_b) begin
        if (wren_b) ram[address_b] <= data_b;
        q_b <= ram[address_b];
    end
endmodule
