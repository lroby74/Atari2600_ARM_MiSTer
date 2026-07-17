// Cyclone-V hardware-safe CDF bridge.
// All large arrays have synchronous, registered ports. No asynchronous RAM read.
// RAM read latency is one clock; bus_ack is returned after the registered data is valid.
// Mixed-port read-during-write is OLD DATA / DON'T CARE on a same-address collision.
// The arbiter prevents logical clients on port B from colliding; ARM owns port A.
`default_nettype none
module cdf_bridge #(
  parameter ROM_FILE="", RAM_FILE="", parameter ROM_DEPTH=32768,
  parameter RAM_DEPTH=8192, parameter DISP_DEPTH=4096,
  parameter bit SIM_TEST_HOOKS=0
)(
  input wire clk,input wire rst,
  input wire [15:0] m6502_addr,input wire [7:0] m6502_din,input wire m6502_we,
  output reg [7:0] m6502_dout,
  input wire [31:0] bus_addr,input wire [31:0] bus_wdata,output reg [31:0] bus_rdata,
  input wire [3:0] bus_be,input wire bus_we,input wire [1:0] bus_sz,input wire bus_req,
  output reg bus_ack,input wire [31:0] dbg_pc,input wire [31:0] dbg_r0,
  input wire [1:0] family_sel,
  input wire rom_load_we,input wire [14:0] rom_load_addr,input wire [7:0] rom_load_data,
  output wire [31:0] dbg_comms_ptr,output wire [3:0] dbg_bank,
  output wire [7:0] dbg_mode,output wire [7:0] dbg_call_fn,
  output wire [3:0] cb_fn,output wire cb_valid,
  output wire [7:0] audio_ch0,output wire [7:0] audio_ch1,output wire [7:0] audio_ch2,
  output wire [31:0] rng_out,output wire [15:0] dbg_ds_base,
  output wire dbg_disp_we,output wire [11:0] dbg_disp_waddr,
  output wire [7:0] dbg_disp250,output wire [4:0] dbg_last_df_idx
);
  localparam ROM_WORDS=ROM_DEPTH/4, RAM_WORDS=RAM_DEPTH/4;
  localparam [2:0] HB_IDLE=3'd0,HB_CPU=3'd1,HB_DF_PTR=3'd2,HB_DF_INC=3'd3,
                   HB_AUDIO_BASE=3'd4,HB_AUDIO_SAMPLE=3'd5;

  wire [15:0] ds_base,ds_inc_base,wf_base,prog_off;
  wire [7:0] amp_stream,jump_mask; wire is_plus; wire [2:0] start_bank;
  wire [4:0] mus_shift,mus_maskbit; wire [31:0] cb0,cb1,cb2,cb3;
  cdf_family_params params(.family_sel(family_sel),.ds_base(ds_base),
    .ds_inc_base(ds_inc_base),.wf_base(wf_base),.amp_stream(amp_stream),
    .jump_mask(jump_mask),.is_plus(is_plus),.start_bank(start_bank),
    .prog_off(prog_off),.mus_shift(mus_shift),.mus_maskbit(mus_maskbit),
    .cb0(cb0),.cb1(cb1),.cb2(cb2),.cb3(cb3));

  // True-dual-port ROM, primitiva M10K esplicita. Porta A=ARM/loader
  // (priorita' al loader), porta B=6502/data-fetcher, sola lettura.
  // Note: DONT_CARE R-D-W is safe as arbitration prevents same-address collision.
  wire [31:0] rom_q_a, rom_q_b; reg [12:0] rom_addr_b;
  wire        rom_wren_a  = rom_load_we | (bus_req && bus_we && bus_addr[31:28]==4'h0);
  wire [12:0] rom_addr_a  = rom_load_we ? rom_load_addr[14:2] : bus_addr[14:2];
  wire [31:0] rom_data_a  = rom_load_we ? {4{rom_load_data}}  : bus_wdata;
  wire [3:0]  rom_be_a    = rom_load_we ? (4'b0001 << rom_load_addr[1:0]) : bus_be;
  
  cdf_rom_m10k #(.INIT_FILE(ROM_FILE)) rom_i (
    .clk(clk),
    .addr_a(rom_addr_a), .data_a(rom_data_a), .byteena_a(rom_be_a),
    .wren_a(rom_wren_a), .q_a(rom_q_a),
    .addr_b(rom_addr_b), .q_b(rom_q_b)
  );

  // Harmony RAM is an explicit altsyncram wrapper in synthesis.
  wire [31:0] harmony_q_a,harmony_q_b; reg [10:0] harmony_addr_b;
  reg harmony_we_b; reg [3:0] harmony_be_b; reg [31:0] harmony_wdata_b;
  wire harmony_we_a = bus_req && bus_we && (bus_addr[31:28]==4'h4);
  harmony_m10k_tdp #(.INIT_FILE(RAM_FILE)) harmony_ram_i (
    .clk(clk),
    .addr_a(bus_addr[12:2]), .data_a(bus_wdata), .byteena_a(bus_be),
    .wren_a(harmony_we_a), .q_a(harmony_q_a),
    .addr_b(harmony_addr_b), .data_b(harmony_wdata_b), .byteena_b(harmony_be_b),
    .wren_b(harmony_we_b), .q_b(harmony_q_b)
  );

  // Display RAM, primitiva M10K esplicita. Porta A=scrittura utente/clear
  // sequenziale, porta B=lettura data-fetcher.
  reg [11:0] clear_addr,display_rd_addr; reg clear_active;
  wire [7:0] display_q;
  reg display_user_we; reg [11:0] display_user_addr; reg [7:0] display_user_data;
  reg disp_we_r; reg [11:0] disp_waddr_r;
  wire        disp_wren_a = clear_active | display_user_we;
  wire [11:0] disp_addr_a = clear_active ? clear_addr : display_user_addr;
  wire [7:0]  disp_data_a = clear_active ? 8'd0 : display_user_data;

  cdf_display_m10k display_i (
    .clk(clk),
    .addr_a(disp_addr_a), .data_a(disp_data_a), .wren_a(disp_wren_a), .q_a(),
    .addr_b(display_rd_addr), .q_b(display_q)
  );

  wire m6502_cs=(m6502_addr[15:12]==4'h1); wire [11:0] off=m6502_addr[11:0];
  reg [3:0] bank_reg; reg [7:0] mode_reg,call_function,fast_fetch_imm;
  reg [31:0] comms_ptr,lfsr; reg [7:0] last_rng,last_df_fetch; reg [4:0] last_df_idx;
  reg [7:0] music_freq[0:2],audio_out[0:2];
  reg [31:0] music_counter[0:2],audio_base_ptr;
  reg [31:0] audio_addr_calc;
  reg [2:0] audio_phase, audio_phase_d; reg [9:0] audio_div;
  reg [2:0] hb_owner,hb_owner_d; reg df_pending,df_wait_display,df_wait_rom;
  reg [31:0] df_ptr; reg [4:0] df_idx; reg [12:0] df_rom_addr;
  reg [11:0] cpu_off_d; reg cpu_pending;
  wire fast_fetch_en=((mode_reg&8'h0f)==0);
  wire cpu_read=m6502_cs && !m6502_we;

  // Port-B request generation. Priority: CPU write/read, DF, periodic audio.
  always @* begin
    harmony_addr_b=11'd0; harmony_we_b=1'b0; harmony_be_b=4'b0000;
    harmony_wdata_b=32'd0; audio_addr_calc=32'd0;
    rom_addr_b=df_wait_rom ? df_rom_addr : (({9'd0,bank_reg}*13'd1024)+{3'd0,off[11:2]});
    hb_owner=HB_IDLE;
    if(m6502_cs && m6502_we && off<12'h040) begin
      hb_owner=HB_CPU; harmony_addr_b={1'b0,off[11:2]}; harmony_we_b=1'b1;
      harmony_be_b=(4'b0001<<off[1:0]); harmony_wdata_b={4{m6502_din}};
    end else if(cpu_read && off<12'h040) begin
      hb_owner=HB_CPU; harmony_addr_b={1'b0,off[11:2]};
    end else if(df_pending) begin
      hb_owner=HB_DF_PTR;
      harmony_addr_b=ds_base[12:2]+{6'd0,df_idx};
    end else if(audio_phase!=0) begin
      if(audio_phase==1) begin hb_owner=HB_AUDIO_BASE; harmony_addr_b=wf_base[12:2]; end
      else begin
        hb_owner=HB_AUDIO_SAMPLE;
        audio_addr_calc=audio_base_ptr+(music_counter[audio_phase-2]>>mus_shift);
        harmony_addr_b=audio_addr_calc[12:2];
      end
    end
  end

  integer k;
  always @(posedge clk) begin
    hb_owner_d<=hb_owner; display_user_we<=1'b0; disp_we_r<=1'b0;
    if(rst) begin
      clear_active<=1'b1; clear_addr<=12'd0; bank_reg<={1'b0,start_bank}; mode_reg<=8'd0;
      call_function<=0; comms_ptr<=0; fast_fetch_imm<=0; lfsr<=32'hdeadbeef;
      last_rng<=8'hef; last_df_fetch<=0; last_df_idx<=0; df_pending<=0;
      df_wait_display<=0; df_wait_rom<=0; cpu_pending<=0; m6502_dout<=0;
      audio_phase<=0; audio_phase_d<=0; audio_div<=0; audio_base_ptr<=0; df_rom_addr<=0;
      for(k=0;k<3;k=k+1) begin music_freq[k]<=8'd0;
        music_counter[k]<=32'd0; audio_out[k]<=8'd0; end
    end else begin
      if(clear_active) begin
        if(clear_addr==12'hfff) clear_active<=0; else clear_addr<=clear_addr+1'b1;
      end
      // 6502 control writes; synchronous address is the current bus address.
      if(m6502_cs && m6502_we) begin
        case(off)
          12'hff0: begin
            display_user_addr<=is_plus ? comms_ptr[27:16] : comms_ptr[31:20];
            display_user_data<=m6502_din; display_user_we<=!clear_active;
            disp_waddr_r<=is_plus ? comms_ptr[27:16] : comms_ptr[31:20]; disp_we_r<=!clear_active;
            comms_ptr<=comms_ptr+(is_plus?32'h00010000:32'h00100000);
          end
          12'hff1: comms_ptr<=is_plus ? ((comms_ptr<<8)&32'hff000000)|({24'd0,m6502_din}<<16)
                                      : ((comms_ptr<<8)&32'hf0000000)|({24'd0,m6502_din}<<20);
          12'hff2: mode_reg<=m6502_din; 12'hff3: call_function<=m6502_din;
          12'hff4: bank_reg<=is_plus?4'd0:4'd6; 12'hff5: bank_reg<=is_plus?4'd1:4'd0;
          12'hff6: bank_reg<=is_plus?4'd2:4'd1; 12'hff7: bank_reg<=is_plus?4'd3:4'd2;
          12'hff8: bank_reg<=is_plus?4'd4:4'd3; 12'hff9: bank_reg<=is_plus?4'd5:4'd4;
          12'hffa: bank_reg<=is_plus?4'd6:4'd5; 12'hffb: bank_reg<=is_plus?4'd0:4'd6;
          12'hffc: if(SIM_TEST_HOOKS) fast_fetch_imm<=m6502_din;
        endcase
      end
      // CPU read response one clock after registered memory output selection.
      cpu_pending<=cpu_read; cpu_off_d<=off;
      if(cpu_pending) begin
        if(cpu_off_d<12'h040) case(cpu_off_d[1:0])
          0:m6502_dout<=harmony_q_b[7:0];1:m6502_dout<=harmony_q_b[15:8];
          2:m6502_dout<=harmony_q_b[23:16];3:m6502_dout<=harmony_q_b[31:24];endcase
        else if(cpu_off_d>=12'hff0 && cpu_off_d<=12'hffb) begin
          case(cpu_off_d) 12'hff2:m6502_dout<=mode_reg;12'hff3:m6502_dout<=call_function;
            default:m6502_dout<={4'd0,bank_reg};endcase
        end else if(cpu_off_d==12'hffd) m6502_dout<=last_df_fetch;
        else if(cpu_off_d==12'hffe) m6502_dout<=last_rng;
        else case(cpu_off_d[1:0]) 0:m6502_dout<=rom_q_b[7:0];1:m6502_dout<=rom_q_b[15:8];
          2:m6502_dout<=rom_q_b[23:16];3:m6502_dout<=rom_q_b[31:24];endcase
      end
      if(cpu_read && off==12'hffd && fast_fetch_en && !clear_active) begin
        df_pending<=1; df_idx<=fast_fetch_imm[4:0]; last_df_idx<=fast_fetch_imm[4:0];
      end
      if(hb_owner_d==HB_DF_PTR) begin
        df_ptr<=harmony_q_b; df_pending<=0;
        if((is_plus?harmony_q_b[27:16]:harmony_q_b[31:20])<DISP_DEPTH) begin
          display_rd_addr<=is_plus?harmony_q_b[27:16]:harmony_q_b[31:20]; df_wait_display<=1;
        end else begin df_rom_addr<=harmony_q_b[14:2]; df_wait_rom<=1; end
      end
      if(df_wait_display) begin last_df_fetch<=display_q; df_wait_display<=0; end
      if(df_wait_rom) begin case(df_ptr[1:0])0:last_df_fetch<=rom_q_b[7:0];1:last_df_fetch<=rom_q_b[15:8];
        2:last_df_fetch<=rom_q_b[23:16];3:last_df_fetch<=rom_q_b[31:24];endcase df_wait_rom<=0;end
      // Three audio samples are serialized on consecutive port-B cycles.
      // A new four-cycle sequence starts every 1024 clocks (~48.8 kHz at 50 MHz).
      audio_phase_d<=audio_phase;
      audio_div<=audio_div+1'b1;
      if(audio_div==0 && audio_phase==0) audio_phase<=1;
      else if(audio_phase!=0) begin
        if(audio_phase==4) audio_phase<=0; else audio_phase<=audio_phase+1'b1;
      end
      if(hb_owner_d==HB_AUDIO_BASE) audio_base_ptr<=harmony_q_b;
      if(hb_owner_d==HB_AUDIO_SAMPLE) begin
        case(audio_phase_d) 2:audio_out[0]<=harmony_q_b[7:0];
          3:audio_out[1]<=harmony_q_b[7:0]; 4:audio_out[2]<=harmony_q_b[7:0]; default:; endcase
      end
      for(k=0;k<3;k=k+1) music_counter[k]<=music_counter[k]+{24'd0,music_freq[k]};
      if(m6502_cs&&!m6502_we&&off==12'hffe) begin last_rng<=lfsr[7:0]|8'h01; lfsr<=(lfsr>>1)^(lfsr[0]?32'h80000057:32'd0);end
      if(SIM_TEST_HOOKS && m6502_cs&&m6502_we&&off==12'h038) music_freq[0]<=m6502_din;
    end
  end

  // ARM response: request edge -> synchronous RAM edge -> ack/data next cycle.
  reg arm_pending; reg [3:0] arm_region_d;
  always @(posedge clk) begin
    if(rst) begin arm_pending<=0;bus_ack<=0;bus_rdata<=0;arm_region_d<=0;end
    else begin
      bus_ack<=arm_pending; arm_pending<=bus_req&&!arm_pending; if(bus_req&&!arm_pending) arm_region_d<=bus_addr[31:28];
      if(arm_pending) bus_rdata<=(arm_region_d==0)?rom_q_a:(arm_region_d==4)?harmony_q_a:0;
    end
  end

  assign cb_fn=(dbg_pc==cb0)?4'd0:(dbg_pc==cb1)?4'd1:(dbg_pc==cb2)?4'd2:(dbg_pc==cb3)?4'd3:4'd15;
  assign cb_valid=(cb_fn!=4'd15); assign dbg_comms_ptr=comms_ptr; assign dbg_bank=bank_reg;
  assign dbg_mode=mode_reg;assign dbg_call_fn=call_function;assign rng_out=lfsr;
  assign dbg_ds_base=ds_base;assign dbg_disp_we=disp_we_r;assign dbg_disp_waddr=disp_waddr_r;
  assign dbg_disp250=8'd0;assign dbg_last_df_idx=last_df_idx;
  assign audio_ch0=audio_out[0];assign audio_ch1=audio_out[1];assign audio_ch2=audio_out[2];
endmodule
`default_nettype wire
