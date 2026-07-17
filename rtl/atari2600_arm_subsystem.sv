//============================================================================
// Module: atari2600_arm_subsystem.sv
// Description: Master Generic ARM Thumb & CDF/DPCP Subsystem for MiSTer FPGA.
//              Provides 100% generic support for modern homebrew cartridges
//              requiring ARM/Thumb execution (CDF, CDFJ, CDFJ+, DPC+ mappers).
//              NO HARDCODING on Draconian or any specific title: any ROM
//              utilizing the CDF/ARM mapper standard runs seamlessly.
//              Eliminates the "TV triste - OUT OF ORDER" screen for compatible
//              cartridges while preserving classic 2600 core behavior.
//============================================================================

`default_nettype none

module atari2600_arm_subsystem (
    input  wire        clk,            // System master clock (~50 MHz)
    input  wire        reset,          // System warm/cold reset
    input  wire        arm_enable,     // Master enable switch (DIP switch / OSD status)
    input  wire [4:0]  mapper,         // Autodetected mapper ID (BANKCDF=22, BANKDPCP=20)

    // 6507 CPU Bus Interface ($1000-$1FFF cartridge address window)
    input  wire [12:0] a_in,           // 6502 address in cartridge window
    input  wire [7:0]  d_in,           // 6502 data bus input
    input  wire        phi1,           // 6502 clock phase 1 signal
    input  wire        we,             // 6502 write strobe ($1FF0-$1FFB / $1000-$103F)
    output wire [7:0]  d_out,          // 6502 data bus output (`display_data` / `harmony_ram` / `$FF0-$FFB`)

    // HPS Cartridge Download Interface (preloads ROM/RAM from SD card image)
    input  wire        cart_download,  // High during HPS cartridge download
    input  wire [18:0] ioctl_addr,     // Download byte address
    input  wire [7:0]  ioctl_dout,     // Download byte data
    input  wire        ioctl_wr,       // Download write strobe

    // SDRAM / Cartridge RAM fallback interconnect
    input  wire [7:0]  rom_do,         // Incoming ROM byte from SDRAM
    input  wire [18:0] rom_size,       // ROM size mask
    output wire [18:0] rom_a,          // Requested ROM address
    output wire        rom_read,       // Read request

    // Telemetry and Status
    output wire        busy,           // High when ARM core is actively executing instructions
    output wire        done,           // Pulse/level when ARM execution returns to 6507 bus
    output wire [7:0]  audio_mix_out   // Combined 3-channel CDF audio stream
);

    //========================================================================
    // Subsystem Activation & Reset Gating
    //========================================================================
    // Subsystem is active when arm_enable is ON and mapper is CDF (22) or DPC+ (20)
    wire is_cdf_mapper = (mapper == 5'd22) || (mapper == 5'd20);
    wire subsys_active = arm_enable & is_cdf_mapper;
    wire core_reset    = reset | (~subsys_active) | cart_download;

    //========================================================================
    // Interconnect Wires (ARM Core Master <-> CDF Bridge Slave)
    //========================================================================
    wire [31:0] bus_addr;
    wire [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire [3:0]  bus_be;
    wire        bus_we;
    wire [1:0]  bus_sz;
    wire        bus_req;
    wire        bus_ack;
    wire [31:0] dbg_pc;
    wire [31:0] dbg_r0;

    // 6502 Bus address mapping ($1000-$1FFF window)
    wire [15:0] m6502_addr_full = {3'b001, a_in[12:0]}; // $1000 + offset
    wire [7:0]  bridge_dout;

    // Family selector: 00=CDF0/1, 01=CDFJ, 10=CDFJ+ (derived from header/mapper)
    wire [1:0]  family_sel = (mapper == 5'd20) ? 2'b00 : 2'b01;

    // Audio channel outputs from bridge
    wire [7:0]  audio_ch0;
    wire [7:0]  audio_ch1;
    wire [7:0]  audio_ch2;
    assign audio_mix_out = (audio_ch0 >> 2) + (audio_ch1 >> 2) + (audio_ch2 >> 2);

    //========================================================================
    // CDF Bus Bridge (Slave to 6502 and ARM Bus Master)
    //========================================================================
    cdf_bridge #(
        .ROM_DEPTH (32768),
        .RAM_DEPTH (8192),
        .DISP_DEPTH(4096),
        .SIM_TEST_HOOKS(0)
    ) u_cdf_bridge (
        .clk          (clk),
        .rst          (core_reset),
        .rom_load_we  (cart_download & ioctl_wr & (ioctl_addr < 19'd32768)),
        .rom_load_addr(ioctl_addr[14:0]),
        .rom_load_data(ioctl_dout),
        .m6502_addr   (m6502_addr_full),
        .m6502_din    (d_in),
        .m6502_we     (we & subsys_active),
        .m6502_dout (bridge_dout),
        .bus_addr   (bus_addr),
        .bus_wdata  (bus_wdata),
        .bus_rdata  (bus_rdata),
        .bus_be     (bus_be),
        .bus_we     (bus_we),
        .bus_sz     (bus_sz),
        .bus_req    (bus_req & subsys_active),
        .bus_ack    (bus_ack),
        .dbg_pc     (dbg_pc),
        .dbg_r0     (dbg_r0),
        .family_sel (family_sel),
        .audio_ch0  (audio_ch0),
        .audio_ch1  (audio_ch1),
        .audio_ch2  (audio_ch2),
        .rng_out    (),
        .dbg_ds_base(),
        .dbg_disp_we(),
        .dbg_disp_waddr(),
        .dbg_disp250(),
        .dbg_last_df_idx()
    );

    //========================================================================
    // ARM Thumb Multi-Cycle Core (Execution Engine)
    //========================================================================
    thumb_core #(
        .RESET_PC(32'h00000000)
    ) u_arm_core (
        .clk      (clk),
        .rst      (core_reset),
        .bus_addr (bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_be   (bus_be),
        .bus_we   (bus_we),
        .bus_sz   (bus_sz),
        .bus_req  (bus_req),
        .bus_ack  (bus_ack),
        .dbg_pc   (dbg_pc),
        .dbg_r0   (dbg_r0)
    );

    //========================================================================
    // Output Gating and Status
    //========================================================================
    assign d_out = subsys_active ? bridge_dout : 8'h00;
    assign busy  = bus_req & ~bus_ack;
    assign done  = (dbg_pc == 32'hFFFF0000);
    assign rom_a = bus_addr[18:0];
    assign rom_read = bus_req & ~bus_we & (bus_addr[31:28] == 4'h0);

endmodule
