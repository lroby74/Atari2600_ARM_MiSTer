//============================================================================
// Module: softcore_thumb_core
// Description: Top-level core for ARM/Thumb CDF/CDFJ+ multi-cycle softcore.
//              Integrates FETCH, DECODE, EXEC, MEM, WRITEBACK, Control Unit,
//              ALU, Regfile, Memory Interface, and CDF Facade.
//============================================================================

module softcore_thumb_core (
    // System
    input         clk,
    input         rst_n,

    // Status
    output        core_busy,
    output        core_fault,
    output        core_returned,
    input         core_halt,

    // Memory bus
    output [31:0] mem_addr,
    input  [31:0] mem_rdata,
    output [31:0] mem_wdata,
    output        mem_read,
    output        mem_write,
    output [3:0]  mem_byte_en,
    input         mem_ready,

    // CDF interface
    input         cdf_callfn,
    input         cdf_setmode,
    input  [7:0]  cdf_func_code,
    output        cdf_busy,
    output        cdf_done,

    // Debug
    output [31:0] dbg_pc,
    output [2:0]  dbg_state,
    output [4:0]  dbg_cpsr,
    input  [3:0]  dbg_reg_sel,
    output [31:0] dbg_reg_val
);

    //========================================================================
    // Parameters
    //========================================================================
    parameter MAIN_ENTRY_ADDR = 32'h00002548;  // Draconian default

    //========================================================================
    // Internal wires
    //========================================================================

    // Control Unit outputs
    wire        state_fetch;
    wire        state_decode;
    wire        state_exec;
    wire        state_mem;
    wire        state_wb;
    wire        pc_load_entry;
    wire        pc_increment;

    // FETCH <-> DECODE
    wire [31:0] ir;
    wire        ir_valid;
    wire        thumb_mode;
    wire        fetch_done;

    // DECODE <-> Control Unit / EXEC / MEM / WRITEBACK
    wire [3:0]  ctrl_alu_op;
    wire [1:0]  ctrl_alu_src_a;
    wire [1:0]  ctrl_alu_src_b;
    wire        ctrl_reg_write;
    wire [1:0]  ctrl_reg_src;
    wire        ctrl_mem_read;
    wire        ctrl_mem_write;
    wire [1:0]  ctrl_mem_size;
    wire        ctrl_branch;
    wire        ctrl_link;
    wire [3:0]  ctrl_cond;
    wire        ctrl_use_imm;
    wire [1:0]  ctrl_shift_op;
    wire [4:0]  ctrl_shift_amt;
    wire        ctrl_update_flags;
    wire [3:0]  ctrl_rd;
    wire [3:0]  ctrl_rn;
    wire [3:0]  ctrl_rs;
    wire [31:0] ctrl_imm;
    wire        ctrl_high_reg;
    wire        decode_done;
    wire        decode_fault;

    // Regfile <-> EXEC / WRITEBACK
    wire [31:0] reg_a_data;
    wire [31:0] reg_b_data;
    wire [31:0] pc_data;
    wire [4:0]  cpsr_data;

    // EXEC outputs
    wire [31:0] alu_result;
    wire [31:0] branch_target;
    wire        branch_taken;
    wire        cond_passed;
    wire [4:0]  flags_out;
    wire        flags_valid;
    wire [31:0] mem_addr_exec;
    wire [31:0] mem_wdata_exec;
    wire        exec_done;

    // MEM outputs
    wire [31:0] mem_rdata_wb;
    wire        mem_rdata_valid;
    wire        mem_done;

    // WRITEBACK outputs
    wire        reg_wr_en;
    wire [3:0]  reg_wr_addr;
    wire [31:0] reg_wr_data;
    wire        pc_wr_en;
    wire [31:0] pc_wr_data;
    wire        lr_wr_en;
    wire [31:0] lr_wr_data;
    wire        cpsr_wr_en;
    wire [4:0]  cpsr_wr_data;
    wire        return_detected;
    wire        wb_done;

    // ALU standalone (for potential direct use)
    wire [31:0] alu_result_standalone;
    wire        alu_n, alu_z, alu_c, alu_v;

    //========================================================================
    // Control Unit
    //========================================================================
    control_unit u_control (
        .clk            (clk),
        .rst_n          (rst_n),
        .core_halt      (core_halt),
        .cdf_callfn     (cdf_callfn),
        .main_entry_addr(MAIN_ENTRY_ADDR),
        .core_busy      (core_busy),
        .core_fault     (core_fault),
        .core_returned  (core_returned),
        .state          (dbg_state),
        .state_fetch    (state_fetch),
        .state_decode   (state_decode),
        .state_exec     (state_exec),
        .state_mem      (state_mem),
        .state_wb       (state_wb),
        .fetch_done     (fetch_done),
        .decode_done    (decode_done),
        .decode_fault   (decode_fault),
        .exec_done      (exec_done),
        .mem_done       (mem_done),
        .wb_done        (wb_done),
        .return_detected(return_detected),
        .pc_load_entry  (pc_load_entry),
        .pc_increment   (pc_increment)
    );

    //========================================================================
    // Regfile
    //========================================================================
    regfile u_regfile (
        .clk          (clk),
        .rst_n        (rst_n),
        .rd_addr_a    (ctrl_rn),
        .rd_data_a    (reg_a_data),
        .rd_addr_b    (ctrl_rs),
        .rd_data_b    (reg_b_data),
        .pc           (pc_data),
        .wr_en        (reg_wr_en),
        .wr_addr      (reg_wr_addr),
        .wr_data      (reg_wr_data),
        .cpsr_wr_en   (cpsr_wr_en),
        .cpsr_wr_data (cpsr_wr_data),
        .cpsr         (cpsr_data),
        .pc_wr_en     (pc_wr_en),
        .pc_wr_data   (pc_wr_data),
        .lr_wr_en     (lr_wr_en),
        .lr_wr_data   (lr_wr_data),
        .thumb_mode   (thumb_mode)
    );

    //========================================================================
    // FETCH Stage
    //========================================================================
    fetch_stage u_fetch (
        .clk          (clk),
        .rst_n        (rst_n),
        .state_fetch  (state_fetch),
        .fetch_done   (fetch_done),
        .pc           (pc_data),
        .mem_addr     (mem_addr),
        .mem_read     (mem_read),
        .mem_rdata    (mem_rdata),
        .mem_ready    (mem_ready),
        .ir           (ir),
        .ir_valid     (ir_valid),
        .thumb_mode   ()
    );

    //========================================================================
    // DECODE Stage
    //========================================================================
    decode_stage u_decode (
        .clk              (clk),
        .rst_n            (rst_n),
        .state_decode     (state_decode),
        .decode_done      (decode_done),
        .decode_fault     (decode_fault),
        .ir               (ir),
        .ir_valid         (ir_valid),
        .thumb_mode       (thumb_mode),
        .pc               (pc_data),
        .ctrl_alu_op      (ctrl_alu_op),
        .ctrl_alu_src_a   (ctrl_alu_src_a),
        .ctrl_alu_src_b   (ctrl_alu_src_b),
        .ctrl_reg_write   (ctrl_reg_write),
        .ctrl_reg_src     (ctrl_reg_src),
        .ctrl_mem_read    (ctrl_mem_read),
        .ctrl_mem_write   (ctrl_mem_write),
        .ctrl_mem_size    (ctrl_mem_size),
        .ctrl_branch      (ctrl_branch),
        .ctrl_link        (ctrl_link),
        .ctrl_cond        (ctrl_cond),
        .ctrl_use_imm     (ctrl_use_imm),
        .ctrl_shift_op    (ctrl_shift_op),
        .ctrl_shift_amt   (ctrl_shift_amt),
        .ctrl_update_flags(ctrl_update_flags),
        .ctrl_rd          (ctrl_rd),
        .ctrl_rn          (ctrl_rn),
        .ctrl_rs          (ctrl_rs),
        .ctrl_imm         (ctrl_imm),
        .ctrl_high_reg    (ctrl_high_reg)
    );

    //========================================================================
    // EXEC Stage
    //========================================================================
    exec_stage u_exec (
        .clk              (clk),
        .rst_n            (rst_n),
        .state_exec       (state_exec),
        .exec_done        (exec_done),
        .ctrl_alu_op      (ctrl_alu_op),
        .ctrl_alu_src_a   (ctrl_alu_src_a),
        .ctrl_alu_src_b   (ctrl_alu_src_b),
        .ctrl_use_imm     (ctrl_use_imm),
        .ctrl_shift_op    (ctrl_shift_op),
        .ctrl_shift_amt   (ctrl_shift_amt),
        .ctrl_update_flags(ctrl_update_flags),
        .ctrl_branch      (ctrl_branch),
        .ctrl_cond        (ctrl_cond),
        .ctrl_high_reg    (ctrl_high_reg),
        .reg_a_data       (reg_a_data),
        .reg_b_data       (reg_b_data),
        .pc_data          (pc_data),
        .sp_data          (reg_a_data),  // Simplified: SP read via reg_a
        .ctrl_imm         (ctrl_imm),
        .cpsr_in          (cpsr_data),
        .alu_result       (alu_result),
        .branch_target    (branch_target),
        .branch_taken     (branch_taken),
        .cond_passed      (cond_passed),
        .flags_out        (flags_out),
        .flags_valid      (flags_valid),
        .mem_addr_exec    (mem_addr_exec),
        .mem_wdata_exec   (mem_wdata_exec),
        .mem_is_store     ()
    );

    //========================================================================
    // MEM Stage
    //========================================================================
    mem_stage u_mem (
        .clk             (clk),
        .rst_n           (rst_n),
        .state_mem       (state_mem),
        .mem_done        (mem_done),
        .ctrl_mem_read   (ctrl_mem_read),
        .ctrl_mem_write  (ctrl_mem_write),
        .ctrl_mem_size   (ctrl_mem_size),
        .ctrl_reg_src    (ctrl_reg_src[0]),
        .mem_addr_exec   (mem_addr_exec),
        .mem_wdata_exec  (mem_wdata_exec),
        .mem_addr        (),
        .mem_wdata       (mem_wdata),
        .mem_rdata       (mem_rdata),
        .mem_read        (),
        .mem_write       (mem_write),
        .mem_byte_en     (mem_byte_en),
        .mem_ready       (mem_ready),
        .mem_rdata_wb    (mem_rdata_wb),
        .mem_rdata_valid (mem_rdata_valid)
    );

    //========================================================================
    // WRITEBACK Stage
    //========================================================================
    writeback_stage u_wb (
        .clk             (clk),
        .rst_n           (rst_n),
        .state_wb        (state_wb),
        .wb_done         (wb_done),
        .ctrl_reg_write  (ctrl_reg_write),
        .ctrl_reg_src    (ctrl_reg_src),
        .ctrl_update_flags(ctrl_update_flags),
        .ctrl_branch     (ctrl_branch),
        .ctrl_link       (ctrl_link),
        .ctrl_rd         (ctrl_rd),
        .alu_result      (alu_result),
        .mem_rdata_wb    (mem_rdata_wb),
        .pc_plus_4       (pc_data + (thumb_mode ? 32'd2 : 32'd4)),
        .flags_out       (flags_out),
        .flags_valid     (flags_valid),
        .branch_taken    (branch_taken),
        .branch_target   (branch_target),
        .cond_passed     (cond_passed),
        .reg_wr_en       (reg_wr_en),
        .reg_wr_addr     (reg_wr_addr),
        .reg_wr_data     (reg_wr_data),
        .pc_wr_en        (pc_wr_en),
        .pc_wr_data      (pc_wr_data),
        .lr_wr_en        (lr_wr_en),
        .lr_wr_data      (lr_wr_data),
        .cpsr_wr_en      (cpsr_wr_en),
        .cpsr_wr_data    (cpsr_wr_data),
        .return_detected (return_detected)
    );

    //========================================================================
    // Standalone ALU (for debug/verification)
    //========================================================================
    alu u_alu_standalone (
        .a          (reg_a_data),
        .b          (reg_b_data),
        .shift_amt  (ctrl_shift_amt),
        .shift_op   (ctrl_shift_op),
        .alu_op     (ctrl_alu_op),
        .use_carry  (1'b0),
        .c_in       (cpsr_data[1]),
        .result     (alu_result_standalone),
        .n          (alu_n),
        .z          (alu_z),
        .c          (alu_c),
        .v          (alu_v)
    );

    //========================================================================
    // Debug outputs
    //========================================================================
    assign dbg_pc = pc_data;
    assign dbg_cpsr = cpsr_data;
    assign dbg_reg_val = (dbg_reg_sel == 4'd15) ? pc_data :
                         (dbg_reg_sel == 4'd13) ? reg_a_data :
                         (dbg_reg_sel == 4'd14) ? reg_b_data :
                         reg_a_data;

endmodule
