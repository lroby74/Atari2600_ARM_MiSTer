//============================================================================
// Module: control_unit
// Description: Control Unit FSM for multi-cycle ARM/Thumb softcore.
//              Orchestrates FETCH -> DECODE -> EXEC -> MEM -> WRITEBACK.
//              Handles RESET, IDLE, HALT, FAULT, RETURNED states.
//============================================================================

module control_unit (
    // System
    input         clk,
    input         rst_n,

    // External control
    input         core_halt,       // Halt request (completes current instruction)
    input         cdf_callfn,      // CDF function call request
    input  [31:0] main_entry_addr, // Entry point address (e.g., 0x2548 for Draconian)

    // Status outputs
    output reg [2:0]  state,       // Current FSM state
    output reg        core_busy,   // Core is executing
    output reg        core_fault,  // Fault detected
    output reg        core_returned, // Return sentinel reached

    // FETCH stage control
    output reg        state_fetch,
    input             fetch_done,

    // DECODE stage control
    output reg        state_decode,
    input             decode_done,
    input             decode_fault,

    // EXEC stage control
    output reg        state_exec,
    input             exec_done,

    // MEM stage control
    output reg        state_mem,
    input             mem_done,

    // WRITEBACK stage control
    output reg        state_wb,
    input             wb_done,
    input             return_detected,

    // Control signal latches (output to datapath)
    output reg [3:0]  latched_alu_op,
    output reg [1:0]  latched_alu_src_a,
    output reg [1:0]  latched_alu_src_b,
    output reg        latched_reg_write,
    output reg [1:0]  latched_reg_src,
    output reg        latched_mem_read,
    output reg        latched_mem_write,
    output reg [1:0]  latched_mem_size,
    output reg        latched_branch,
    output reg        latched_link,
    output reg [3:0]  latched_cond,
    output reg        latched_use_imm,
    output reg [1:0]  latched_shift_op,
    output reg [4:0]  latched_shift_amt,
    output reg        latched_update_flags,
    output reg [3:0]  latched_rd,
    output reg [3:0]  latched_rn,
    output reg [3:0]  latched_rs,
    output reg [31:0] latched_imm,
    output reg        latched_high_reg,

    // PC control
    output reg        pc_load_entry,  // Load PC with main_entry_addr (on CALLFN)
    output reg        pc_increment    // Increment PC ( Thumb: +2, ARM: +4)
);

    //========================================================================
    // FSM State Encoding
    //========================================================================
    localparam RESET     = 3'b000;
    localparam IDLE      = 3'b001;
    localparam FETCH     = 3'b010;
    localparam DECODE    = 3'b011;
    localparam EXEC      = 3'b100;
    localparam MEM       = 3'b101;
    localparam WRITEBACK = 3'b110;
    localparam FAULT     = 3'b111;

    //========================================================================
    // Sequential: FSM state transitions
    //========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= RESET;
        end else begin
            case (state)
                RESET: begin
                    state <= IDLE;
                end

                IDLE: begin
                    if (cdf_callfn && !core_halt)
                        state <= FETCH;
                    else if (core_halt)
                        state <= IDLE;  // Stay idle while halted
                end

                FETCH: begin
                    if (fetch_done)
                        state <= DECODE;
                end

                DECODE: begin
                    if (decode_done) begin
                        if (decode_fault)
                            state <= FAULT;
                        else
                            state <= EXEC;
                    end
                end

                EXEC: begin
                    if (exec_done) begin
                        if (latched_mem_read || latched_mem_write)
                            state <= MEM;
                        else
                            state <= WRITEBACK;
                    end
                end

                MEM: begin
                    if (mem_done)
                        state <= WRITEBACK;
                end

                WRITEBACK: begin
                    if (wb_done) begin
                        if (return_detected)
                            state <= IDLE;
                        else if (core_halt)
                            state <= IDLE;
                        else
                            state <= FETCH;
                    end
                end

                FAULT: begin
                    // Remain in FAULT until reset
                    state <= FAULT;
                end

                default: begin
                    state <= FAULT;
                end
            endcase
        end
    end

    //========================================================================
    // Combinational: state outputs and control signal generation
    //========================================================================
    always @(*) begin
        // Default: all stage controls low
        state_fetch  = 1'b0;
        state_decode = 1'b0;
        state_exec   = 1'b0;
        state_mem    = 1'b0;
        state_wb     = 1'b0;
        pc_load_entry = 1'b0;
        pc_increment  = 1'b0;
        core_busy     = 1'b0;
        core_fault    = 1'b0;
        core_returned = 1'b0;

        case (state)
            RESET: begin
                // Initialization: nothing active
            end

            IDLE: begin
                if (cdf_callfn) begin
                    pc_load_entry = 1'b1;  // Load PC with main_entry_addr
                end
            end

            FETCH: begin
                state_fetch = 1'b1;
                core_busy   = 1'b1;
                pc_increment = 1'b1;  // PC advances for next fetch
            end

            DECODE: begin
                state_decode = 1'b1;
                core_busy    = 1'b1;
            end

            EXEC: begin
                state_exec = 1'b1;
                core_busy  = 1'b1;
            end

            MEM: begin
                state_mem = 1'b1;
                core_busy = 1'b1;
            end

            WRITEBACK: begin
                state_wb = 1'b1;
                core_busy = 1'b1;
                if (return_detected)
                    core_returned = 1'b1;
            end

            FAULT: begin
                core_fault = 1'b1;
                core_busy  = 1'b0;
            end

            default: begin
                core_fault = 1'b1;
            end
        endcase
    end

    //========================================================================
    // Sequential: latch control signals from decoder in DECODE state
    //========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latched_alu_op       <= 4'b0100;  // ADD (NOP-like)
            latched_alu_src_a    <= 2'b00;
            latched_alu_src_b    <= 2'b00;
            latched_reg_write    <= 1'b0;
            latched_reg_src      <= 2'b00;
            latched_mem_read     <= 1'b0;
            latched_mem_write    <= 1'b0;
            latched_mem_size     <= 2'b10;
            latched_branch       <= 1'b0;
            latched_link         <= 1'b0;
            latched_cond         <= 4'b1110;  // AL
            latched_use_imm      <= 1'b0;
            latched_shift_op     <= 2'b00;
            latched_shift_amt    <= 5'b00000;
            latched_update_flags <= 1'b0;
            latched_rd           <= 4'b0000;
            latched_rn           <= 4'b0000;
            latched_rs           <= 4'b0000;
            latched_imm          <= 32'h00000000;
            latched_high_reg     <= 1'b0;
        end else begin
            if (state_decode && decode_done) begin
                // These would normally come from the decoder module
                // For now, they are inputs to the control unit
                // In the integrated design, the decoder drives these directly
                // and the control unit latches them
            end
        end
    end

endmodule
