`include "wired0_defines.svh"

function automatic logic[1:0] gen_next_lphr(input logic[1:0] old, input logic direction);
  case(old)
    default : begin
      return {1'b0,direction};
    end
    2'b01 : begin
      return {direction,1'b0};
    end
    2'b10 : begin
      return {direction,1'b1};
    end
    2'b11 : begin
      return {1'b1,direction};
    end
  endcase
endfunction

function automatic logic[6:0] get_tag(input logic[31:0] addr);
  return addr[18:12];
endfunction

function automatic logic[`_WIRED_PARAM_BINFO_LEN-1:0] get_hash(input logic[31:0] addr);
  // return {{addr[12],addr[13],addr[14]} ^ addr[11:9], addr[8:6], addr[5:3]};
  return addr[`_WIRED_PARAM_BINFO_LEN-1+3:3] ^ addr[`_WIRED_PARAM_BINFO_LEN-1+`_WIRED_PARAM_BINFO_LEN+3:`_WIRED_PARAM_BINFO_LEN+3];
endfunction

// 这是 Wired 项目的 PC GEN，实际上是一个两级分支预测器。
// 预测被背靠背的执行，吞吐量为1，每周期均可以处理一条分支指令。
module wired_pcgen (
  `_WIRED_GENERAL_DEFINE,
  input  bpu_correct_t       p_correct_i, // 预测反馈更新

  input  logic               p_ready_i, // 握手信号组
  output logic               p_valid_o,

  output logic  [31:0]       p_pc_o,
  output logic  [1:0]        p_mask_o,
  output bpu_predict_t [1:0] p_predict_o

);

logic tier_id_q;  // tier_id
always_ff @(posedge clk) begin
  if(!rst_n) begin
    tier_id_q <= '0;
  end else if(p_correct_i.redirect) begin
    tier_id_q <= p_correct_i.tid;
  end
end

// -1 周期信号
logic [31:0] npc;  // 下一周期的 PC，纯组合逻辑由预测器得出，用于输入 sram
logic [31:0] pc;   // 本周期输出的 PC，纯寄存器组成。
assign p_pc_o = pc;

typedef struct packed {
  bpu_target_type_e target_type;
  logic conditional_jmp; // 0 / 1 condition
  logic [`_WIRED_PARAM_BHT_HISTORY_LEN-1:0] history;
  logic [6:0] tag;
} branch_info_t;

always_ff @(posedge clk) begin
  if(!rst_n) begin
    pc <= 32'h1c000000;
  end else begin
    if(p_ready_i) begin
      pc <= npc;
    end
  end
end

// SRAM 读取逻辑
/*
NPC -|SRAM|-> {btb, info.history -> l2_cnt}
 |                 |
 |<-_______________|
*/

// BTB, INFORAM 逻辑
logic [1:0][31:0]    btb_rdata;
branch_info_t [1:0] info_rdata;
logic [1:0]          info_p_we;
logic [1:0]          info_n_we;
branch_info_t [1:0] info_wdata;
logic [1:0][1:0]        l2_cnt;
wire l2_we = p_correct_i.true_conditional_jmp && p_correct_i.need_update;
wire [`_WIRED_PARAM_BHT_HISTORY_LEN + `_WIRED_PARAM_BHT_PC_LEN - 1 :0] l2_waddr = {p_correct_i.pc[`_WIRED_PARAM_BHT_PC_LEN - 1 + 3 : 3], p_correct_i.history[`_WIRED_PARAM_BHT_HISTORY_LEN - 1 : 0]};
wire [1:0] l2_wdata = gen_next_lphr(p_correct_i.lphr, p_correct_i.true_taken);
(* ram_style = "distributed" *) reg [1:0] l2_ram [(1 << (`_WIRED_PARAM_BHT_HISTORY_LEN + `_WIRED_PARAM_BHT_PC_LEN)) - 1:0];
always_ff @(posedge clk) begin
  if(l2_we) begin
    l2_ram[l2_waddr] <= l2_wdata;
  end
end
assign l2_cnt[0] = l2_ram[{pc[`_WIRED_PARAM_BHT_PC_LEN - 1 + 3 : 3], info_rdata[0].history[`_WIRED_PARAM_BHT_HISTORY_LEN - 1 : 0]}];
assign l2_cnt[1] = l2_ram[{pc[`_WIRED_PARAM_BHT_PC_LEN - 1 + 3 : 3], info_rdata[1].history[`_WIRED_PARAM_BHT_HISTORY_LEN - 1 : 0]}];
for(genvar p = 0 ; p < 2 ; p += 1) begin
  assign btb_rdata[p][1:0] = '0;
  wired_dpsram # (
    .DATA_WIDTH(30),
    .DATA_DEPTH(1<<(`_WIRED_PARAM_BINFO_LEN)),
    .BYTE_SIZE(30)
  )
  btb_sram (
    .clk0(clk),
    .rst_n0(rst_n),
    .addr0_i(get_hash(npc)),
    .en0_i(p_ready_i),
    .we0_i('0),
    .wdata0_i('0),
    .rdata0_o(btb_rdata[p][31:2]),

    .clk1(clk),
    .rst_n1(rst_n),
    .addr1_i(get_hash(p_correct_i.pc)),
    .en1_i('1),
    .we1_i(p_correct_i.need_update && 
          (p_correct_i.true_target_type inside {BPU_TARGET_CALL, BPU_TARGET_IMM}) &&
          (p_correct_i.pc[2] == p[0])
    ),
    .wdata1_i(p_correct_i.btb_target[31:2]),
    .rdata1_o(/* NOCONNECT */)
  );
  wire [`_WIRED_PARAM_BINFO_LEN-1:0] info_raddr = get_hash(npc);
  wire [`_WIRED_PARAM_BINFO_LEN-1:0] info_waddr = get_hash(info_p_we[p] ? p_correct_i.pc : pc);
  branch_info_t info_raw;
  branch_info_t info_raw_q;
  (* ram_style = "distributed" *) branch_info_t info_ram [(1<<(`_WIRED_PARAM_BINFO_LEN))-1:0];
  always_ff @(posedge clk) begin
    if(info_p_we[p] | info_n_we[p]) begin
      info_ram[info_waddr] <= info_wdata[p];
    end
  end
  assign info_raw = info_ram[info_raddr];
  always_ff @(posedge clk) begin
    if(p_ready_i || p_correct_i.redirect) begin
      info_raw_q <= info_raw;
    end
  end
  assign info_rdata[p] = info_raw_q;
end

// TAKEN 生成逻辑
logic [1:0] branch_need_jmp;
for(genvar p = 0 ; p < 2 ; p += 1) begin
  wire tag_match = info_rdata[p].tag == get_tag(pc);
  always_comb begin
    info_p_we[p] = '0;
    info_n_we[p] = '0;
    info_wdata[p] = info_rdata[p];
    info_wdata[p].history = {info_rdata[p].history[`_WIRED_PARAM_BHT_HISTORY_LEN-2:0], 1'b0};
    branch_need_jmp[p] = '0;
    if(info_rdata[p].target_type != BPU_TARGET_NPC && tag_match) begin
      info_n_we[p] = p_valid_o & p_mask_o[p];
      if(info_rdata[p].target_type != BPU_TARGET_IMM || !info_rdata[p].conditional_jmp) begin
        branch_need_jmp[p] = '1;
        info_wdata[p].history = {info_rdata[p].history[`_WIRED_PARAM_BHT_HISTORY_LEN-2:0], 1'b1};
      end
      else begin
        branch_need_jmp[p] = l2_cnt[p][1];
        info_wdata[p].history = {info_rdata[p].history[`_WIRED_PARAM_BHT_HISTORY_LEN-2:0], l2_cnt[p][1]};
      end
    end
    if(p_correct_i.need_update && p_correct_i.miss && p_correct_i.pc[2] == p[0]) begin
      info_wdata[p].target_type = p_correct_i.true_target_type;
      info_wdata[p].conditional_jmp = p_correct_i.true_conditional_jmp;
      info_wdata[p].history = {p_correct_i.history[`_WIRED_PARAM_BHT_HISTORY_LEN-2:0], p_correct_i.true_taken};
      info_wdata[p].tag = get_tag(p_correct_i.pc);
      info_p_we[p] = '1;
    end
  end
end

// mask 生成逻辑
logic nmask_0_valid;
logic mask_0_valid_q;
assign nmask_0_valid = !npc[2];
always_ff @(posedge clk) begin
  if(!rst_n || p_ready_i) begin
    mask_0_valid_q <= nmask_0_valid;    
  end
end

logic pc_is_cal, pc_is_ret;
// RAS 生成逻辑
logic[7:0][31:0] ras_q;
logic[2:0] ras_w_ptr_q,ras_ptr_q; // TODO:check
logic [31:0] ras_rdata_q;
always_ff @(posedge clk) ras_rdata_q <= ras_q[ras_ptr_q];
always_ff @(posedge clk) begin
  if(!rst_n) begin
    ras_ptr_q <= 0;
    ras_w_ptr_q <= 1;
    ras_q <= 'x;
  end
  else begin
    // TODO: check
    // RETURN MISS: RAS_PTR_Q <= (TRUE_PTR == R_PTR)
    // CALL MISS RAS_PTR_Q <= TRUE_PTR AND ras_q[TRUE_PTR] <= TRUE_TARGET;
    if(p_correct_i.miss || p_correct_i.ras_miss_type) begin
      /* 考虑一下，没有 miss 但是类型估计错误的情况，这时候也需要更新 */
      /* printf */
      ras_w_ptr_q <= p_correct_i.ras_ptr + 3'd1;
      ras_ptr_q <= p_correct_i.ras_ptr;
      if(p_correct_i.true_target_type == BPU_TARGET_CALL) begin
        ras_q[p_correct_i.ras_ptr] <= p_correct_i.pc + 3'd4;
      end
    end
    else begin
      if(pc_is_cal && p_ready_i) begin
        ras_q[ras_w_ptr_q] <= {pc[31:3], 3'b000} + ((mask_0_valid_q & branch_need_jmp[0]) ? 32'd4 : 32'd8);
        ras_w_ptr_q <= ras_w_ptr_q + 3'd1;
        ras_ptr_q <= ras_ptr_q + 3'd1;
      end
      if(pc_is_ret && p_ready_i) begin
        ras_w_ptr_q <= ras_w_ptr_q - 3'd1;
        ras_ptr_q <= ras_ptr_q - 3'd1;
      end
    end
  end
end

// NPC 生成逻辑
always_comb begin
  p_valid_o = '1;
  npc = {pc[31:3] + 1'd1, 3'b000}; // PC + 8 LOGIC
  p_mask_o = {'1, mask_0_valid_q};
  p_predict_o = '0;
  pc_is_cal = '0;
  pc_is_ret = '0;
  if(mask_0_valid_q && branch_need_jmp[0]) begin
    p_predict_o[0].taken = '1;
    p_mask_o[1] = '0;
    pc_is_cal = info_rdata[0].target_type == BPU_TARGET_CALL;
    pc_is_ret = info_rdata[0].target_type == BPU_TARGET_RETURN;
    npc = info_rdata[0].target_type == BPU_TARGET_RETURN ? ras_rdata_q : btb_rdata[0];
  end else if(branch_need_jmp[1]) begin
    p_predict_o[1].taken = '1;
    pc_is_cal = info_rdata[1].target_type == BPU_TARGET_CALL;
    pc_is_ret = info_rdata[1].target_type == BPU_TARGET_RETURN;
    npc = info_rdata[1].target_type == BPU_TARGET_RETURN ? ras_rdata_q : btb_rdata[1];
  end
  if(p_correct_i.redirect) begin
    p_valid_o = '0;
    npc = p_correct_i.true_target;
  end
  for(integer i = 0 ; i < 2 ; i += 1) begin
    p_predict_o[i].tid = tier_id_q;
    p_predict_o[i].predict_pc = npc;
    p_predict_o[i].lphr = l2_cnt[i];
    p_predict_o[i].history = info_rdata[i].history;
    p_predict_o[i].target_type = info_rdata[i].target_type;
    p_predict_o[i].dir_type = info_rdata[i].conditional_jmp;
    p_predict_o[i].ras_ptr = ras_ptr_q;
  end
end

// 调试观察端口
always_ff @(posedge clk) begin
  // if((info_p_we[0] | info_n_we[0]) && (info_p_we[0] ? p_correct_i.pc : pc) == 32'h1c001d08) begin
  //   $display("Get update %b", info_wdata[0].history);
  // end
end

endmodule
