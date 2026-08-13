`default_nettype none

module arm_thumb_cpu (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cpu_en,
    input  wire        n_wait,
    output reg  [31:0] mem_addr,
    input  wire [15:0] mem_din,
    output reg  [31:0] mem_dout,
    output reg         mem_read,
    output reg         mem_write,
    output wire        cpu_seq,
    output wire [31:0] cpsr_out
);

    reg [31:0] regs [0:15];
    reg [15:0] instr_reg;
    reg [31:0] cpsr;
    reg [2:0]  state;
    localparam FETCH=0, DECODE=1, EXECUTE=2, MULTI_TRANSFER=3, MEM_SETUP=4, MEM_STROBE=5;

    reg [7:0]  reg_mask;
    reg [3:0]  transfer_idx;
    reg [31:0] current_ptr;
    reg [3:0]  base_rn;
    reg        is_ldm;

    assign cpu_seq = (state >= MULTI_TRANSFER);
    assign cpsr_out = cpsr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= FETCH; mem_read <= 0; mem_write <= 0; mem_addr <= 0; mem_dout <= 0;
            for (integer i = 0; i < 16; i = i + 1) regs[i] <= 32'h0;
            regs[13] <= 32'h40001FFF; current_ptr <= 32'h0;
        end else if (n_wait) begin
            case (state)
                FETCH: if (cpu_en) begin
                    mem_addr <= regs[15]; mem_read <= 1; mem_write <= 0; state <= DECODE;
                end
                DECODE: begin
                    instr_reg <= mem_din; mem_read <= 0; state <= EXECUTE;
                end
                EXECUTE: begin
                    if ((instr_reg & 16'hF000) == 16'hC000) begin
                        is_ldm <= instr_reg[11];
                        base_rn <= {1'b0, instr_reg[10:8]};
                        reg_mask <= instr_reg[7:0];
                        current_ptr <= regs[instr_reg[10:8]];
                        transfer_idx <= 0; state <= MULTI_TRANSFER;
                    end else begin
                        regs[15] <= regs[15] + 2; state <= FETCH;
                    end
                end
                MULTI_TRANSFER: begin
                    if (reg_mask == 8'h0 || transfer_idx == 8) begin
                        if (!(is_ldm && (instr_reg & (1 << base_rn[2:0])))) regs[base_rn] <= current_ptr;
                        regs[15] <= regs[15] + 2; state <= FETCH;
                    end else if (reg_mask[transfer_idx[2:0]]) begin
                        mem_addr <= current_ptr;
                        if (!is_ldm) mem_dout <= regs[transfer_idx[2:0]];
                        state <= MEM_SETUP;
                    end else begin
                        transfer_idx <= transfer_idx + 1;
                    end
                end
                MEM_SETUP: begin
                    if (is_ldm) mem_read <= 1; else mem_write <= 1;
                    state <= MEM_STROBE;
                end
                MEM_STROBE: begin
                    if (is_ldm) regs[transfer_idx[2:0]] <= {mem_din, mem_din};
                    mem_read <= 0; mem_write <= 0;
                    current_ptr <= current_ptr + 4;
                    reg_mask[transfer_idx[2:0]] <= 1'b0;
                    transfer_idx <= transfer_idx + 1;
                    state <= MULTI_TRANSFER;
                end
            endcase
        end
    end
endmodule
