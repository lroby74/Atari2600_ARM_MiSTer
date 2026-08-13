`default_nettype none

module harmony_bus_interface (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_din,
    output reg  [31:0] cpu_dout,
    input  wire        cpu_read,
    input  wire        cpu_write,
    input  wire        cpu_seq,
    output wire        n_wait,       
    output wire [14:0] ram_addr,
    output wire [31:0] ram_dout,
    input  wire [31:0] ram_din,
    output wire        ram_we,
    output reg [31:0] music_freq [0:2],
    output reg [15:0] music_wave [0:2]
);
    wire is_ram = (cpu_addr[31:15] == 17'h08000);
    wire is_dpc_reg = (cpu_addr[31:7] == 25'h000020);

    reg wait_state;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wait_state <= 0;
        else if (is_dpc_reg && (cpu_read || cpu_write) && !wait_state) wait_state <= 1;
        else wait_state <= 0;
    end
    assign n_wait = !wait_state;

    assign ram_addr = cpu_addr[14:0];
    assign ram_dout = cpu_din;
    assign ram_we   = cpu_write && is_ram;

    reg [31:0] df_pointers [0:15];
    reg [31:0] df_increments [0:15];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integer i;
            for (i = 0; i < 16; i = i + 1) begin
                df_pointers[i] <= 32'h0;
                df_increments[i] <= 32'h0;
            end
        end else if (is_dpc_reg && n_wait) begin
            if (cpu_write) begin
                case (cpu_addr[6:2])
                    5'h00, 5'h01, 5'h02, 5'h03, 5'h04, 5'h05, 5'h06, 5'h07: df_pointers[cpu_addr[4:2]] <= cpu_din;
                    5'h08, 5'h09, 5'h0A, 5'h0B, 5'h0C, 5'h0D, 5'h0E, 5'h0F: df_increments[cpu_addr[4:2]] <= cpu_din;
                    5'h10, 5'h11, 5'h12: music_freq[cpu_addr[3:2]] <= cpu_din;
                    5'h13, 5'h14, 5'h15: music_wave[cpu_addr[3:2]] <= cpu_din[15:0];
                endcase
            end else if (cpu_read && !cpu_addr[6]) begin
                df_pointers[cpu_addr[5:2]] <= df_pointers[cpu_addr[5:2]] + df_increments[cpu_addr[5:2]];
            end
        end
    end

    always @(*) begin
        if (is_ram) cpu_dout = ram_din;
        else if (is_dpc_reg) begin
            if (!cpu_addr[6]) cpu_dout = df_pointers[cpu_addr[5:2]];
            else cpu_dout = 32'h0;
        end else cpu_dout = 32'hFFFFFFFF;
    end
endmodule
