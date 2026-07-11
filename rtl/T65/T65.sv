// Top-level T65 65xx-compatible CPU core.
/* verilator lint_off IMPORTSTAR */
/* verilator lint_off UNUSED */
import T65_Pack::*;
module T65 (
	input  [1:0]  Mode,
	input         BCD_en,
	input         Res_n,
	input         Enable,
	input         Clk,
	input         Rdy,
	input         Abort_n,
	input         IRQ_n,
	input         NMI_n,
	input         SO_n,
	output        R_W_n,
	output        Sync,
	output        EF,
	output        MF,
	output        XF,
	output        ML_n,
	output        VP_n,
	output        VDA,
	output        VPA,
	output reg [23:0] A,
	input  [7:0]  DI,
	output [7:0]  DO,
	output [63:0] Regs,
	output T_t65_dbg DEBUG,
	output        NMI_ack
);

// CPU registers and internal buses.
reg  [7:0] ABC, X, Y;
reg  [7:0] P, AD, DL;
wire [7:0] PwithB;
reg  [7:0] BAH;
reg  [8:0] BAL;
reg  [7:0] PBR, DBR;
reg  [15:0] PC, S;
reg  EF_i, MF_i, XF_i;
reg  [7:0] IR;
reg  [2:0] MCycle;
reg  [7:0] DO_r;
reg  [1:0] Mode_r;
reg        BCD_en_r;
T_ALU_OP ALU_Op_r;
T_Write_Data Write_Data_r;
T_Set_Addr_To Set_Addr_To_r;
wire [8:0] PCAdder;
reg  RstCycle, IRQCycle, NMICycle, IRQReq, NMIReq;
reg  SO_n_o, IRQ_n_o, NMI_n_o, NMIAct;
wire Break;
reg  [7:0] BusA;
reg  [7:0] BusA_r, BusB, BusB_r;
wire [7:0] ALU_Q, P_Out;
T_Lcycle LCycle;
T_ALU_OP ALU_Op;
T_Set_BusA_To Set_BusA_To;
T_Set_Addr_To Set_Addr_To;
T_Write_Data Write_Data;
wire [1:0] Jump, BAAdd, BAQuirk;
wire BreakAtNA, ADAdd, AddY, PCAdd, Inc_S, Dec_S;
wire LDA, LDP, LDX, LDY, LDS, LDDI, LDALU, LDAD, LDBAL, LDBAH, SaveP, Write;
reg  Res_n_i, Res_n_d;
wire rdy_mod_set = ((MCycle == Cycle_3 && IR != 8'h93) || (MCycle == Cycle_4 && IR == 8'h93)) && !Rdy;
reg  rdy_mod, NMI_entered;
wire really_rdy = Rdy | ~WRn_i;
reg  WRn_i;

// Direct output aliases.
assign NMI_ack = NMIAct;
assign Sync = MCycle == Cycle_sync;
assign EF = EF_i;
assign MF = MF_i;
assign XF = XF_i;
assign R_W_n = WRn_i;
assign ML_n = IR[7:6] != 2'b10 && IR[2:1] == 2'b11 && MCycle[2:1] != 2'b00 ? 1'b0 : 1'b1;
assign VP_n = IRQCycle && (MCycle == Cycle_5 || MCycle == Cycle_6) ? 1'b0 : 1'b1;
assign VDA = Set_Addr_To_r != Set_Addr_To_PBR;
assign VPA = ~Jump[1];
assign DO = DO_r;
assign Regs = {PC, S, P, Y, X, ABC};
assign DEBUG.I = IR;
assign DEBUG.A = ABC;
assign DEBUG.X = X;
assign DEBUG.Y = Y;
assign DEBUG.S = S[7:0];
assign DEBUG.P = P;

// Microcode decoder instantiation.
T65_MCode mcode (
	.Mode(Mode_r),
	.IR(IR),
	.MCycle(MCycle),
	.P(P),
	.Rdy_mod(rdy_mod),
	.LCycle(LCycle),
	.ALU_Op(ALU_Op),
	.Set_BusA_To(Set_BusA_To),
	.Set_Addr_To(Set_Addr_To),
	.Write_Data(Write_Data),
	.Jump(Jump),
	.BAAdd(BAAdd),
	.BAQuirk(BAQuirk),
	.BreakAtNA(BreakAtNA),
	.ADAdd(ADAdd),
	.AddY(AddY),
	.PCAdd(PCAdd),
	.Inc_S(Inc_S),
	.Dec_S(Dec_S),
	.LDA(LDA),
	.LDP(LDP),
	.LDX(LDX),
	.LDY(LDY),
	.LDS(LDS),
	.LDDI(LDDI),
	.LDALU(LDALU),
	.LDAD(LDAD),
	.LDBAL(LDBAL),
	.LDBAH(LDBAH),
	.SaveP(SaveP),
	.Write(Write)
);

// Arithmetic logic unit instantiation.
T65_ALU alu (
	.Mode(Mode_r),
	.BCD_en(BCD_en_r),
	.Op(ALU_Op_r),
	.BusA(BusA_r),
	.BusB(BusB),
	.P_In(P),
	.P_Out(P_Out),
	.Q(ALU_Q)
);

// Reset input sampler.
always @(posedge Clk or negedge Res_n) begin
	if (!Res_n) begin
		Res_n_i <= 1'b0;
		Res_n_d <= 1'b0;
	end else begin
		Res_n_i <= Res_n_d;
		Res_n_d <= 1'b1;
	end
end

// Program counter, instruction, mode and cycle-control registers.
always @(posedge Clk or negedge Res_n_i) begin
	if (!Res_n_i) begin
		PC <= 16'h0000;
		IR <= 8'h00;
		S <= 16'h0000;
		PBR <= 8'h00;
		DBR <= 8'h00;
		Mode_r <= 2'b00;
		BCD_en_r <= 1'b1;
		ALU_Op_r <= ALU_OP_BIT;
		Write_Data_r <= Write_Data_DL;
		Set_Addr_To_r <= Set_Addr_To_PBR;
		WRn_i <= 1'b1;
		EF_i <= 1'b1;
		MF_i <= 1'b1;
		XF_i <= 1'b1;
		NMICycle <= 1'b0;
		IRQCycle <= 1'b0;
		rdy_mod <= 1'b0;
	end else if (Enable) begin
		if (MCycle == Cycle_sync) rdy_mod <= 1'b0;
		else if (rdy_mod_set) rdy_mod <= 1'b1;

		if (really_rdy) begin
			WRn_i <= ~Write | RstCycle;
			PBR <= 8'hff;
			DBR <= 8'hff;
			EF_i <= 1'b0;
			MF_i <= 1'b0;
			XF_i <= 1'b0;

			if (MCycle == Cycle_sync) begin
				Mode_r <= Mode;
				BCD_en_r <= BCD_en;
				if (!IRQReq && !NMIReq) PC <= PC + 16'd1;
				IR <= (IRQReq || NMIReq) ? 8'h00 : DI;
				IRQCycle <= 1'b0;
				NMICycle <= 1'b0;
				if (NMIReq) NMICycle <= 1'b1;
				else if (IRQReq) IRQCycle <= 1'b1;
				if (LDS) S[7:0] <= ALU_Q;
			end

			ALU_Op_r <= ALU_Op;
			Write_Data_r <= Write_Data;
			Set_Addr_To_r <= Break ? Set_Addr_To_PBR : Set_Addr_To;

			if (Inc_S) S <= S + 16'd1;
			if (Dec_S && (!RstCycle || Mode == 2'b00)) S <= S - 16'd1;
			if (IR == 8'h00 && MCycle == Cycle_1 && !IRQCycle && !NMICycle) PC <= PC + 16'd1;

			case (Jump)
				2'b01: PC <= PC + 16'd1;
				2'b10: PC <= {DI, DL};
				2'b11: begin
					if (PCAdder[8]) PC[15:8] <= !DL[7] ? PC[15:8] + 8'd1 : PC[15:8] - 8'd1;
					PC[7:0] <= PCAdder[7:0];
				end
				default: ;
			endcase
		end
	end
end

// Program counter relative adder.
assign PCAdder = PCAdd ? {1'b0, PC[7:0]} + {DL[7], DL} : {1'b0, PC[7:0]};

// Processor status and accumulator/index register updater.
always @(posedge Clk or negedge Res_n_i) begin
	reg [7:0] tmpP;

	if (!Res_n_i) begin
		P <= 8'h00;
		ABC <= 8'h00;
		X <= 8'h00;
		Y <= 8'h00;
		SO_n_o <= 1'b1;
	end else begin
		tmpP = P;
		if (Enable && really_rdy) begin
			if (MCycle == Cycle_sync) begin
				if (LDA) ABC <= ALU_Q;
				if (LDX) X <= ALU_Q;
				if (LDY) Y <= ALU_Q;
				if (LDA || LDX || LDY) tmpP = P_Out;
			end
			if (SaveP) tmpP = P_Out;
			if (LDP) tmpP = ALU_Q;
			if (IR[4:0] == 5'b11000) begin
				case (IR[7:5])
					3'b000: tmpP[Flag_C] = 1'b0;
					3'b001: tmpP[Flag_C] = 1'b1;
					3'b010: tmpP[Flag_I] = 1'b0;
					3'b011: tmpP[Flag_I] = 1'b1;
					3'b101: tmpP[Flag_V] = 1'b0;
					3'b110: tmpP[Flag_D] = 1'b0;
					3'b111: tmpP[Flag_D] = 1'b1;
					default: ;
				endcase
			end
			tmpP[Flag_B] = 1'b1;
			if (IR == 8'h00 && MCycle == Cycle_4 && !RstCycle) tmpP[Flag_I] = 1'b1;
			if (RstCycle) begin
				tmpP[Flag_I] = 1'b1;
				tmpP[Flag_D] = 1'b0;
			end
			tmpP[Flag_1] = 1'b1;
			P <= tmpP;
		end

		SO_n_o <= SO_n;
		if (SO_n_o && !SO_n) P[Flag_V] <= 1'b1;
	end
end

// Address and data latch updater.
always @(posedge Clk or negedge Res_n_i) begin
	if (!Res_n_i) begin
		BusA_r <= 8'h00;
		BusB <= 8'h00;
		BusB_r <= 8'h00;
		AD <= 8'h00;
		BAL <= 9'h000;
		BAH <= 8'h00;
		DL <= 8'h00;
		NMI_entered <= 1'b0;
	end else if (Enable && really_rdy) begin
		NMI_entered <= 1'b0;
		BusA_r <= BusA;
		BusB <= DI;

		if (Set_Addr_To_r == Set_Addr_To_PBR || Set_Addr_To_r == Set_Addr_To_ZPG) BusB_r <= DI + 8'd1;

		case (BAAdd)
			2'b01: begin
				AD <= AD + 8'd1;
				BAL <= BAL + 9'd1;
			end
			2'b10: BAL <= {1'b0, BAL[7:0]} + {1'b0, BusA};
			2'b11: if (BAL[8]) begin
				case (BAQuirk)
					2'b00: BAH <= BAH + 8'd1;
					2'b01: BAH <= (BAH + 8'd1) & DO_r;
					2'b10: BAH <= DO_r;
					default: ;
				endcase
			end
			default: ;
		endcase

		if (ADAdd) AD <= AD + (AddY ? Y : X);

		if (IR == 8'h00) begin
			BAL <= 9'h1ff;
			BAH <= 8'hff;
			if (RstCycle) BAL[2:0] <= 3'b100;
			else if (NMICycle || (NMIAct && MCycle == Cycle_4) || NMI_entered) begin
				BAL[2:0] <= 3'b010;
				if (MCycle == Cycle_4) NMI_entered <= 1'b1;
			end else begin
				BAL[2:0] <= 3'b110;
			end
			if (Set_Addr_To_r == Set_Addr_To_BA) BAL[0] <= 1'b1;
		end

		if (LDDI) DL <= DI;
		if (LDALU) DL <= ALU_Q;
		if (LDAD) AD <= DI;
		if (LDBAL) BAL[7:0] <= DI;
		if (LDBAH) BAH <= DI;
	end
end

// Branch and page-crossing break detector.
assign Break = (BreakAtNA & ~BAL[8]) | (PCAdd & ~PCAdder[8]);

// ALU input bus mux.
always @* begin
	case (Set_BusA_To)
		Set_BusA_To_DI:  BusA = DI;
		Set_BusA_To_ABC: BusA = ABC;
		Set_BusA_To_X:   BusA = X;
		Set_BusA_To_Y:   BusA = Y;
		Set_BusA_To_S:   BusA = S[7:0];
		Set_BusA_To_P:   BusA = P;
		Set_BusA_To_DA:  BusA = ABC & DI;
		Set_BusA_To_DAO: BusA = (ABC | 8'hee) & DI;
		Set_BusA_To_DAX: BusA = (ABC | 8'hee) & DI & X;
		Set_BusA_To_AAX: BusA = ABC & X;
		default:         BusA = 8'hxx;
	endcase
end

// CPU address output mux.
always @* begin
	case (Set_Addr_To_r)
		Set_Addr_To_SP:  A = {16'h0001, S[7:0]};
		Set_Addr_To_ZPG: A = {DBR, 8'h00, AD};
		Set_Addr_To_BA:  A = {8'h00, BAH, BAL[7:0]};
		default:         A = {PBR, PC[15:8], PCAdder[7:0]};
	endcase
end

// Stack status value with corrected break flag.
assign PwithB = (IRQCycle || NMICycle) ? (P & 8'hef) : P;

// CPU data output mux.
always @* begin
	case (Write_Data_r)
		Write_Data_DL:  DO_r = DL;
		Write_Data_ABC: DO_r = ABC;
		Write_Data_X:   DO_r = X;
		Write_Data_Y:   DO_r = Y;
		Write_Data_S:   DO_r = S[7:0];
		Write_Data_P:   DO_r = PwithB;
		Write_Data_PCL: DO_r = PC[7:0];
		Write_Data_PCH: DO_r = PC[15:8];
		Write_Data_AX:  DO_r = ABC & X;
		Write_Data_AXB: DO_r = ABC & X & BusB_r;
		Write_Data_XB:  DO_r = X & BusB_r;
		Write_Data_YB:  DO_r = Y & BusB_r;
		default:        DO_r = 8'hxx;
	endcase
end

// Main machine-cycle and interrupt-state updater.
always @(posedge Clk or negedge Res_n_i) begin
	if (!Res_n_i) begin
		MCycle <= Cycle_1;
		RstCycle <= 1'b1;
		NMIAct <= 1'b0;
		IRQReq <= 1'b0;
		NMIReq <= 1'b0;
		IRQ_n_o <= 1'b1;
		NMI_n_o <= 1'b1;
	end else if (Enable) begin
		if (really_rdy) begin
			if (MCycle == LCycle || Break) begin
				MCycle <= Cycle_sync;
				RstCycle <= 1'b0;
			end else begin
				MCycle <= MCycle + 3'd1;
			end

			if (IR[4:0] != 5'b10000 || Jump != 2'b11) begin
				NMIReq <= NMIAct && IR != 8'h00;
				IRQReq <= !IRQ_n_o && !P[Flag_I];
			end
		end

		IRQ_n_o <= IRQ_n;
		NMI_n_o <= NMI_n;
		if (NMI_n_o && !NMI_n) NMIAct <= 1'b1;
		if (NMI_entered) NMIAct <= 1'b0;
	end
end

endmodule
/* verilator lint_on UNUSED */
/* verilator lint_on IMPORTSTAR */
