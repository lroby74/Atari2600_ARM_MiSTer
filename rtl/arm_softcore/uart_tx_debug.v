//============================================================================
// Module: uart_tx_debug
// Description: Synthesizable UART Telemetry Transmitter for Hardware Debug
//              in Chapter 19/21. Transmits core PC, FSM State, and CPSR flags
//              over serial UART (`UART_TXD`) at 115200 baud (8N1) when triggered
//              by button press (`KEY[1]`) or core return event.
//============================================================================

`timescale 1ns/1ps

module uart_tx_debug (
    input         clk,          // 50 MHz core clock
    input         rst_n,        // Synchronous active-low reset
    input         trigger,      // Pulse high to trigger packet transmission
    input  [31:0] dbg_pc,       // Core Program Counter
    input  [2:0]  dbg_state,    // Core FSM State
    input  [4:0]  dbg_cpsr,     // CPSR flags {N, Z, C, V, T}
    output reg    uart_tx,      // Serial TX output pin
    output reg    tx_busy       // High while transmitting
);

    //========================================================================
    // Baud Rate Generator: 115200 baud @ 50 MHz clock -> 434 cycles/bit
    //========================================================================
    localparam BAUD_DIV = 16'd434;
    reg [15:0] baud_cnt;
    wire       bit_tick = (baud_cnt == BAUD_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 16'd0;
        end else if (tx_busy) begin
            if (bit_tick)
                baud_cnt <= 16'd0;
            else
                baud_cnt <= baud_cnt + 1'b1;
        end else begin
            baud_cnt <= 16'd0;
        end
    end

    //========================================================================
    // ASCII Hex Conversion Helper Function
    //========================================================================
    function [7:0] nibble_to_hex;
        input [3:0] nibble;
        begin
            nibble_to_hex = (nibble <= 4'h9) ? (8'h30 + nibble) : (8'h37 + nibble);
        end
    endfunction

    //========================================================================
    // Packet Buffer: "P:xxxxxxxx S:x\r\n" = 14 bytes = 140 serial bits
    //========================================================================
    reg [7:0] packet [0:13];
    reg [3:0] byte_idx;
    reg [3:0] bit_idx;
    reg [7:0] shift_reg;

    localparam TX_IDLE  = 2'd0;
    localparam TX_START = 2'd1;
    localparam TX_DATA  = 2'd2;
    localparam TX_STOP  = 2'd3;
    reg [1:0] tx_state;

    //========================================================================
    // FSM: Packet Serialization and Transmission
    //========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx  <= 1'b1; // Idle serial line is high (Mark)
            tx_busy  <= 1'b0;
            tx_state <= TX_IDLE;
            byte_idx <= 4'd0;
            bit_idx  <= 4'd0;
            shift_reg <= 8'h00;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    uart_tx <= 1'b1;
                    if (trigger && !tx_busy) begin
                        tx_busy  <= 1'b1;
                        byte_idx <= 4'd0;
                        bit_idx  <= 4'd0;

                        // Format ASCII packet: "P:xxxxxxxx S:x\r\n"
                        packet[0]  <= 8'h50; // 'P'
                        packet[1]  <= 8'h3A; // ':'
                        packet[2]  <= nibble_to_hex(dbg_pc[31:28]);
                        packet[3]  <= nibble_to_hex(dbg_pc[27:24]);
                        packet[4]  <= nibble_to_hex(dbg_pc[23:20]);
                        packet[5]  <= nibble_to_hex(dbg_pc[19:16]);
                        packet[6]  <= nibble_to_hex(dbg_pc[15:12]);
                        packet[7]  <= nibble_to_hex(dbg_pc[11:8]);
                        packet[8]  <= nibble_to_hex(dbg_pc[7:4]);
                        packet[9]  <= nibble_to_hex(dbg_pc[3:0]);
                        packet[10] <= 8'h20; // ' '
                        packet[11] <= nibble_to_hex({1'b0, dbg_state});
                        packet[12] <= 8'h0D; // '\r'
                        packet[13] <= 8'h0A; // '\n'

                        shift_reg <= 8'h50; // Load first byte ('P')
                        tx_state  <= TX_START;
                    end else begin
                        tx_busy <= 1'b0;
                    end
                end

                TX_START: begin
                    uart_tx <= 1'b0; // Start bit is low (Space)
                    if (bit_tick) begin
                        tx_state <= TX_DATA;
                        bit_idx  <= 4'd0;
                    end
                end

                TX_DATA: begin
                    uart_tx <= shift_reg[0]; // LSB first transmission
                    if (bit_tick) begin
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        if (bit_idx == 4'd7) begin
                            tx_state <= TX_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end
                end

                TX_STOP: begin
                    uart_tx <= 1'b1; // Stop bit is high (Mark)
                    if (bit_tick) begin
                        if (byte_idx == 4'd13) begin
                            // Entire 14-byte packet transmitted
                            tx_state <= TX_IDLE;
                            tx_busy  <= 1'b0;
                        end else begin
                            // Proceed to next byte
                            byte_idx  <= byte_idx + 1'b1;
                            shift_reg <= packet[byte_idx + 1'b1];
                            tx_state  <= TX_START;
                        end
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
