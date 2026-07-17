//============================================================================
// Module: fpga_pll
// Description: Clock Conditioning, Reset Synchronizer & Heartbeat Generator
//              for Intel / Altera Cyclone V and MAX 10 FPGA targets.
//              Provides clean 50 MHz core clock, 3-stage synchronized rst_n,
//              1 Hz heartbeat tick, and 1 kHz UI/debounce tick.
//============================================================================

`timescale 1ns/1ps

module fpga_pll (
    input  clk_50m_in,      // 50 MHz external oscillator input pin
    input  ext_rst_n,       // External active-low reset button pin (KEY[0])

    output clk_core,        // Synchronous 50 MHz core clock output
    output rst_n_sync,      // Fully synchronized active-low core reset
    output reg heartbeat,   // 1 Hz LED heartbeat toggle
    output reg ui_tick      // 1 kHz strobe for debouncing and telemetry
);

    //========================================================================
    // Clock Routing / ALTPLL Wrapper
    //========================================================================
    // In default 50 MHz core operation, clk_core is directly buffered.
    // If ALTPLL IP is generated in Quartus for higher/lower frequencies,
    // this assign is replaced by the altpll instance instantiation.
    assign clk_core = clk_50m_in;

    //========================================================================
    // 3-Stage Reset Synchronizer (Metastability Elimination)
    //========================================================================
    reg [2:0] rst_sync_shift;

    always @(posedge clk_core or negedge ext_rst_n) begin
        if (!ext_rst_n) begin
            rst_sync_shift <= 3'b000;
        end else begin
            rst_sync_shift <= {rst_sync_shift[1:0], 1'b1};
        end
    end

    assign rst_n_sync = rst_sync_shift[2];

    //========================================================================
    // Heartbeat & UI Tick Counters (50 MHz reference)
    //========================================================================
    reg [25:0] hb_counter;
    reg [15:0] ui_counter;

    localparam HB_HALF_PERIOD = 26'd25_000_000; // 0.5 sec toggle at 50 MHz
    localparam UI_PERIOD      = 16'd50_000;     // 1 ms (1 kHz) at 50 MHz

    always @(posedge clk_core or negedge rst_n_sync) begin
        if (!rst_n_sync) begin
            hb_counter <= 26'd0;
            ui_counter <= 16'd0;
            heartbeat  <= 1'b0;
            ui_tick    <= 1'b0;
        end else begin
            // Heartbeat toggle every 25,000,000 cycles (1 Hz LED blink)
            if (hb_counter >= HB_HALF_PERIOD - 1) begin
                hb_counter <= 26'd0;
                heartbeat  <= ~heartbeat;
            end else begin
                hb_counter <= hb_counter + 1'b1;
            end

            // UI tick strobe every 50,000 cycles (1 kHz)
            if (ui_counter >= UI_PERIOD - 1) begin
                ui_counter <= 16'd0;
                ui_tick    <= 1'b1;
            end else begin
                ui_counter <= ui_counter + 1'b1;
                ui_tick    <= 1'b0;
            end
        end
    end

endmodule
