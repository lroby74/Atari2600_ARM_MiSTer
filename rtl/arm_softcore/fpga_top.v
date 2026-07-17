//============================================================================
// Module: fpga_top
// Description: Top-Level FPGA Integration Wrapper for Intel / Altera Quartus
//              Targeting Terasic DE10-Nano (MiSTer Cyclone V) and DE10-Lite.
//              Integrates softcore_thumb_core, mem_interface, cdf_facade,
//              fpga_bram (M10K), fpga_rom (M10K), fpga_pll, and uart_tx_debug.
//============================================================================

`timescale 1ns/1ps

module fpga_top (
    // Clock and Reset inputs
    input        CLOCK_50,       // 50 MHz physical clock pin (PIN_V11 on DE10-Nano)
    input  [1:0] KEY,            // Active-low push buttons: KEY[0]=Reset, KEY[1]=Trigger/Step

    // Switches and LED status outputs
    input  [3:0] SW,             // DIP switches SW[3:0] for debug register selection (dbg_reg_sel)
    output [7:0] LEDR,           // Status LEDs: Busy, Fault, Returned, State, Heartbeat, UART

    // Serial Debug Telemetry
    output       UART_TXD        // UART 115200 baud debug transmitter pin
);

    //========================================================================
    // Clock domain and Reset synchronization
    //========================================================================
    wire clk_core;
    wire rst_n_sync;
    wire heartbeat_tick;
    wire ui_tick;

    fpga_pll u_pll (
        .clk_50m_in (CLOCK_50),
        .ext_rst_n  (KEY[0]),
        .clk_core   (clk_core),
        .rst_n_sync (rst_n_sync),
        .heartbeat  (heartbeat_tick),
        .ui_tick    (ui_tick)
    );

    //========================================================================
    // Button Debounce and Pulse Generation (KEY[1] -> Trigger / Callfn Pulse)
    //========================================================================
    reg [2:0] key1_sync;
    reg       key1_debounced;
    reg       key1_debounced_d;
    wire      key1_pressed_pulse;

    always @(posedge clk_core or negedge rst_n_sync) begin
        if (!rst_n_sync) begin
            key1_sync        <= 3'b111;
            key1_debounced   <= 1'b1;
            key1_debounced_d <= 1'b1;
        end else begin
            // 3-stage synchronizer for external button pin
            key1_sync <= {key1_sync[1:0], KEY[1]};

            // Sample debounced state when ui_tick (1 kHz) strobes
            if (ui_tick) begin
                key1_debounced <= key1_sync[2];
            end
            key1_debounced_d <= key1_debounced;
        end
    end

    // Falling edge detector (button press goes 1 -> 0)
    assign key1_pressed_pulse = key1_debounced_d & ~key1_debounced;

    //========================================================================
    // Core Status and Bus Interconnects
    //========================================================================
    wire        core_busy;
    wire        core_fault;
    wire        core_returned;

    wire [31:0] core_mem_addr;
    wire [31:0] core_mem_rdata;
    wire [31:0] core_mem_wdata;
    wire        core_mem_read;
    wire        core_mem_write;
    wire [3:0]  core_mem_byte_en;
    wire        core_mem_ready;

    wire [31:0] bus_addr;
    wire [31:0] bus_wdata;
    reg  [31:0] bus_rdata;
    wire        bus_read;
    wire        bus_write;
    wire [3:0]  bus_byte_en;
    wire        bus_ready;

    wire        bank_wr_en;
    wire [7:0]  bank_wr_data;
    wire [7:0]  bank_reg;

    wire [31:0] dbg_pc;
    wire [2:0]  dbg_state;
    wire [4:0]  dbg_cpsr;
    wire [31:0] dbg_reg_val;

    wire        cdf_busy;
    wire        cdf_done;
    wire [7:0]  cdf_func_code = 8'd0; // Default: RUN_INIT (or selectable via additional DIP switches)

    //========================================================================
    // Device Under Test: ARM/Thumb Core Top-Level
    //========================================================================
    softcore_thumb_core #(
        .MAIN_ENTRY_ADDR(32'h00002548) // Default Draconian / testarm entry point
    ) u_core (
        .clk          (clk_core),
        .rst_n        (rst_n_sync),
        .core_busy    (core_busy),
        .core_fault   (core_fault),
        .core_returned(core_returned),
        .core_halt    (1'b0),
        .mem_addr     (core_mem_addr),
        .mem_rdata    (core_mem_rdata),
        .mem_wdata    (core_mem_wdata),
        .mem_read     (core_mem_read),
        .mem_write    (core_mem_write),
        .mem_byte_en  (core_mem_byte_en),
        .mem_ready    (core_mem_ready),
        .cdf_callfn   (key1_pressed_pulse),
        .cdf_setmode  (1'b0),
        .cdf_func_code(cdf_func_code),
        .cdf_busy     (cdf_busy),
        .cdf_done     (cdf_done),
        .dbg_pc       (dbg_pc),
        .dbg_state    (dbg_state),
        .dbg_cpsr     (dbg_cpsr),
        .dbg_reg_sel  (SW[3:0]),
        .dbg_reg_val  (dbg_reg_val)
    );

    //========================================================================
    // Memory Interface (Alignment & Bank Translation)
    //========================================================================
    wire [1:0] core_mem_size = (core_mem_byte_en == 4'b1111) ? 2'b10 :
                               (core_mem_byte_en == 4'b0011 || core_mem_byte_en == 4'b1100) ? 2'b01 : 2'b00;

    mem_interface u_mem_if (
        .clk          (clk_core),
        .rst_n        (rst_n_sync),
        .mem_en       (core_mem_read | core_mem_write),
        .mem_rw       (core_mem_write),
        .mem_addr     (core_mem_addr),
        .mem_wdata    (core_mem_wdata),
        .mem_size     (core_mem_size),
        .mem_rdata    (core_mem_rdata),
        .mem_ready    (core_mem_ready),
        .bus_addr     (bus_addr),
        .bus_wdata    (bus_wdata),
        .bus_rdata    (bus_rdata),
        .bus_read     (bus_read),
        .bus_write    (bus_write),
        .bus_byte_en  (bus_byte_en),
        .bus_ready    (bus_ready),
        .bank_wr_en   (bank_wr_en),
        .bank_wr_data (bank_wr_data),
        .bank_reg     (bank_reg)
    );

    //========================================================================
    // Memory Window Decoding (RAM vs ROM vs CDF Facade)
    //========================================================================
    wire is_ram_window = (bus_addr >= 32'h40000000) && (bus_addr < 32'h40002000);
    wire is_rom_window = (bus_addr < 32'h00010000);

    wire [31:0] bram_rdata;
    wire [31:0] rom_rdata;
    wire [31:0] cdf_facade_rdata;

    // Synchronous M10K Block RAM (8 KB Shared Memory)
    fpga_bram #(
        .DATA_WIDTH(32),
        .ADDR_WIDTH(11),
        .INIT_FILE ("")
    ) u_bram (
        .clk     (clk_core),
        .en      ((bus_read || bus_write) && is_ram_window),
        .we      (bus_write && is_ram_window),
        .addr    (bus_addr[12:2]),
        .wdata   (bus_wdata),
        .byte_en (bus_byte_en),
        .rdata   (bram_rdata)
    );

    // Synchronous M10K Block ROM (64 KB Program Memory)
    fpga_rom #(
        .DATA_WIDTH(32),
        .ADDR_WIDTH(14),
        .INIT_FILE ("testarm.hex")
    ) u_rom (
        .clk     (clk_core),
        .en      (bus_read && is_rom_window),
        .addr    (bus_addr[15:2]),
        .rdata   (rom_rdata)
    );

    // CDF Facade (Queue Header & Datastream I/O Registers)
    cdf_facade u_cdf (
        .clk          (clk_core),
        .rst_n        (rst_n_sync),
        .ram_addr     (bus_addr),
        .ram_wdata    (bus_wdata),
        .ram_rdata    (cdf_facade_rdata),
        .ram_we       (bus_write && is_ram_window),
        .ram_be       (bus_byte_en),
        .cdf_callfn   (key1_pressed_pulse),
        .cdf_setmode  (1'b0),
        .cdf_func_code(cdf_func_code),
        .cdf_busy     (cdf_busy),
        .cdf_done     (cdf_done),
        .ds_index     (6'd0),
        .ds_data      (),
        .ds_ptr       (),
        .ds_inc       (),
        .audio_reg0   (),
        .audio_reg1   (),
        .queue_func   (),
        .queue_mode   (),
        .queue_frame  ()
    );

    // Read Data Muxing
    always @(*) begin
        if (is_ram_window) begin
            // In shared RAM region, return M10K BRAM data or Facade registers
            bus_rdata = bram_rdata;
        end else if (is_rom_window) begin
            bus_rdata = rom_rdata;
        end else begin
            bus_rdata = 32'h00000000;
        end
    end

    // Single-cycle ready response for internal M10K memory access
    assign bus_ready = 1'b1;

    //========================================================================
    // UART Debug Telemetry Transmitter (115200 baud)
    //========================================================================
    wire uart_tx_busy;

    uart_tx_debug u_uart (
        .clk       (clk_core),
        .rst_n     (rst_n_sync),
        .trigger   (key1_pressed_pulse | core_returned),
        .dbg_pc    (dbg_pc),
        .dbg_state (dbg_state),
        .dbg_cpsr  (dbg_cpsr),
        .uart_tx   (UART_TXD),
        .tx_busy   (uart_tx_busy)
    );

    //========================================================================
    // Status LED Output Mapping
    //========================================================================
    assign LEDR[0]   = core_busy;
    assign LEDR[1]   = core_fault;
    assign LEDR[2]   = core_returned;
    assign LEDR[3]   = heartbeat_tick;
    assign LEDR[6:4] = dbg_state[2:0];
    assign LEDR[7]   = uart_tx_busy;

endmodule
