//============================================================================
// Module: fetch_stage
// Description: FETCH micro-stage for multi-cycle ARM/Thumb softcore.
//              Reads instruction from memory at PC and loads it into IR.
//============================================================================

module fetch_stage (
    // System
    input         clk,
    input         rst_n,

    // Control from Control Unit
    input         state_fetch,     // High when FSM is in FETCH state
    output        fetch_done,      // Pulse high when IR is loaded

    // PC input
    input  [31:0] pc,

    // Memory interface
    output [31:0] mem_addr,        // Address = PC
    output        mem_read,        // Read enable
    input  [31:0] mem_rdata,       // Data from memory
    input         mem_ready,       // Memory read completed

    // Output to DECODE stage
    output reg [31:0] ir,          // Instruction Register
    output reg        ir_valid,    // IR contains valid instruction
    output reg        thumb_mode   // 1=Thumb, 0=ARM (latched from PC[0])
);

    //========================================================================
    // Combinational outputs to memory
    //========================================================================
    assign mem_addr = pc;
    assign mem_read = state_fetch;

    // Fetch done when we are in FETCH and memory responds
    assign fetch_done = state_fetch && mem_ready;

    //========================================================================
    // Sequential: load IR when fetch completes
    //========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ir        <= 32'h00000000;
            ir_valid  <= 1'b0;
            thumb_mode <= 1'b0;
        end else begin
            if (fetch_done) begin
                // For Thumb: memory returns halfword in lower 16 bits
                // For ARM: memory returns full 32-bit word
                // We load the raw data; decoder will interpret based on thumb_mode
                ir        <= mem_rdata;
                ir_valid  <= 1'b1;
                thumb_mode <= pc[0];  // Thumb if PC[0]=1, ARM if PC[0]=0
            end else if (!state_fetch) begin
                // Clear valid when leaving FETCH (optional, helps debug)
                ir_valid  <= 1'b0;
            end
        end
    end

endmodule
