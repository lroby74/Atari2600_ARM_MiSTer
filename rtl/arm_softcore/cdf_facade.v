//============================================================================
// Module: cdf_facade
// Description: CDF/CDFJ+ facade mapped into shared RAM.
//              Manages Queue header, QPTR, QINC, display-data, audio.
//============================================================================

module cdf_facade (
    input         clk,
    input         rst_n,

    // RAM interface (memory-mapped)
    input  [31:0] ram_addr,
    input  [31:0] ram_wdata,
    output [31:0] ram_rdata,
    input         ram_we,
    input  [3:0]  ram_be,

    // Explicit CDF signals (from 6507 / testbench)
    input         cdf_callfn,
    input         cdf_setmode,
    input  [7:0]  cdf_func_code,
    output        cdf_busy,
    output        cdf_done,

    // Datastream interface
    input  [5:0]  ds_index,        // 0-33
    output [7:0]  ds_data,
    output [31:0] ds_ptr,
    output [31:0] ds_inc,

    // Audio output
    output [31:0] audio_reg0,
    output [31:0] audio_reg1,

    // Queue header direct access
    output [7:0]  queue_func,
    output [7:0]  queue_mode,
    output [7:0]  queue_frame
);

    //========================================================================
    // CDF memory map (relative to RAM_BASE = 0x40000000)
    //========================================================================
    localparam QUEUE_OFFSET   = 32'h00000000;  // Queue header: FUNC, SWCHA, SWCHB, INPT4, MODE, FRAME
    localparam QPTR_OFFSET    = 32'h000000A0;  // QPTR[0] at 0x400000A0
    localparam QINC_OFFSET    = 32'h00000128;  // QINC[0] at 0x40000128
    localparam WAVEFORM_OFFSET= 32'h000001B0;  // Waveform registers
    localparam DD_OFFSET      = 32'h00000800;  // Display-data

    //========================================================================
    // Internal RAM for CDF region (8KB: 0x40000000 - 0x40001FFF)
    //========================================================================
    reg [7:0] cdf_ram [0:8191];

    //========================================================================
    // Word read helper
    //========================================================================
    wire [31:0] word_addr = ram_addr - 32'h40000000;
    wire [12:0] byte_addr = word_addr[12:0];

    //========================================================================
    // RAM read (combinational)
    //========================================================================
    reg [31:0] rdata_comb;
    always @(*) begin
        if (word_addr < 8192) begin
            rdata_comb = {cdf_ram[byte_addr+3], cdf_ram[byte_addr+2],
                         cdf_ram[byte_addr+1], cdf_ram[byte_addr]};
        end else begin
            rdata_comb = 32'h00000000;
        end
    end
    assign ram_rdata = rdata_comb;

    //========================================================================
    // RAM write (sequential with byte-enable)
    //========================================================================
    always @(posedge clk) begin
        if (ram_we && word_addr < 8192) begin
            if (ram_be[0]) cdf_ram[byte_addr+0] <= ram_wdata[7:0];
            if (ram_be[1]) cdf_ram[byte_addr+1] <= ram_wdata[15:8];
            if (ram_be[2]) cdf_ram[byte_addr+2] <= ram_wdata[23:16];
            if (ram_be[3]) cdf_ram[byte_addr+3] <= ram_wdata[31:24];
        end
    end

    //========================================================================
    // Queue header direct outputs
    //========================================================================
    assign queue_func  = cdf_ram[QUEUE_OFFSET + 0];
    assign queue_mode  = cdf_ram[QUEUE_OFFSET + 4];
    assign queue_frame = cdf_ram[QUEUE_OFFSET + 5];

    //========================================================================
    // Datastream interface
    //========================================================================
    wire [12:0] qptr_addr = QPTR_OFFSET + {ds_index, 2'b00};
    wire [12:0] qinc_addr = QINC_OFFSET + {ds_index, 2'b00};

    assign ds_ptr = {cdf_ram[qptr_addr+3], cdf_ram[qptr_addr+2],
                    cdf_ram[qptr_addr+1], cdf_ram[qptr_addr]};
    assign ds_inc = {cdf_ram[qinc_addr+3], cdf_ram[qinc_addr+2],
                    cdf_ram[qinc_addr+1], cdf_ram[qinc_addr]};

    // Datastream read (pointer-based from flash, not directly from RAM)
    // This is a simplified model; actual datastream fetch would use
    // the flash interface with auto-incrementing QPTR
    assign ds_data = 8'h00;  // Placeholder: actual fetch happens via flash

    //========================================================================
    // Audio registers
    //========================================================================
    assign audio_reg0 = {cdf_ram[WAVEFORM_OFFSET+3], cdf_ram[WAVEFORM_OFFSET+2],
                        cdf_ram[WAVEFORM_OFFSET+1], cdf_ram[WAVEFORM_OFFSET]};
    assign audio_reg1 = {cdf_ram[WAVEFORM_OFFSET+7], cdf_ram[WAVEFORM_OFFSET+6],
                        cdf_ram[WAVEFORM_OFFSET+5], cdf_ram[WAVEFORM_OFFSET+4]};

    //========================================================================
    // CDF status (simplified)
    //========================================================================
    reg busy_reg;
    reg done_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy_reg <= 1'b0;
            done_reg <= 1'b0;
        end else begin
            if (cdf_callfn) begin
                busy_reg <= 1'b1;
                done_reg <= 1'b0;
            end else if (cdf_done) begin
                busy_reg <= 1'b0;
                done_reg <= 1'b1;
            end
        end
    end

    assign cdf_busy = busy_reg;
    assign cdf_done = done_reg;

endmodule
