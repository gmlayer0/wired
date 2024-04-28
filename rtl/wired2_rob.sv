`include "wired0_defines.svh"



// Valid 表项，四写口六读口，disPatch 及 CDB 均需要写入
typedef logic rob_entry_valid_t;

// Data 表项，双写口，六读口，CDB 写入

typedef word_t rob_entry_data_t;

// Dynamic 表项，双写口双读口，CDB 写入
typedef struct packed {
          // 控制流相关
          lsu_excp_t  excp;
          logic       need_jump;
          logic[31:0] target_addr;

          // 访存流相关
          logic  wrong_forward;
          logic       uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。

          // 浮点相关
          logic          fcc;
          fp_excp_t  fp_excp;
        } rob_entry_dynamic_t;
// ROB 存储表项定义
// Static 表项，双写口双读口，在 disPatch 时写入，保持不变
// 也是从 disPatch 写入到 ROB 的指令静态信息（提交级使用）
typedef struct packed {
          decode_info_rob_t di; // 指令控制信息
          arch_rid_t        wreg;
        //   rob_rid_t [1:0]   rreg;        // 重命名后的读寄存器
          logic             wtier;       // 写寄存器 tier id
          logic[4:0]        op_code;     // CSR 控制信息
          // 注意，op_code 的功能比较丰富，对于跳转指令，用于提供指令类型信息。
          // 对于写寄存器为 1 的分支指令，op_code[4] = '1;  // 函数调用
          // 对于读寄存器1为 1 的分支指令，op_code[3] = '1; // 函数返回
          // 对于 CSR | rdcnt 指令，存储 RJ
          // 对于 INVTLB / CACOP 存储 RD（OP）
          logic[13:0]       csr_id;
          logic[31:0]       pc;
          bpu_predict_t     bpu_predict; // 在 ALU 中仅检查是否跳转，跳转执行由提交级负责
          static_excp_t     excp;
        } rob_entry_static_t;

function automatic rob_entry_t gather_rob(rob_entry_static_t static_i, rob_entry_dynamic_t dynamic_i, rob_entry_data_t data_i, rob_entry_valid_t valid_i);
  rob_entry_t ret;
  ret.valid = valid_i;
  ret.di = static_i.di; // 指令控制信息
  ret.wreg = static_i.wreg;
  ret.wtier = static_i.wtier;       // 写寄存器 tier id
  ret.csr_id = static_i.csr_id;
  ret.op_code = static_i.op_code;       // CSR 控制信息
  ret.pc = static_i.pc;
  ret.bpu_predict = static_i.bpu_predict; // 在 ALU 中仅检查是否跳转，跳转执行由提交级负责
  ret.static_excp = static_i.excp;
  // 控制流相关
  ret.lsu_excp = dynamic_i.excp;
  ret.excp_found = (|dynamic_i.excp) | (|static_i.excp);
  ret.need_jump = dynamic_i.need_jump;
  ret.target_addr = dynamic_i.target_addr;
  // 访存流相关
  ret.wrong_forward = dynamic_i.wrong_forward;
  ret.uncached = dynamic_i.uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。
  // ret.store_buffer = dynamic_i.store_buffer;      // 提交一条 Store_buffer 中的写请求
  // ret.store_conditional = dynamic_i.store_conditional; // 条件写，若未命中，则直接失败并冲刷流水线
  // 浮点相关
  ret.fp_excp = dynamic_i.fp_excp;
  ret.fcc = dynamic_i.fcc;
  // 写回 ARF 的数据
  ret.wdata = data_i;
  return ret;
endfunction

// Fuction module for Wired project
module wired_rob (
    `_WIRED_GENERAL_DEFINE,

    // 连接到 DISPATCH(P) 级别的端口

    // Part 1: ROB 读端口
    // ROB 中读取到的数据（P 级）
    // input  rob_rid_t        [3:0] p_rrrid_i,
    input pipeline_ctrl_p_t [1:0] p_ctrl_i,
    input pipeline_data_t   [1:0] p_data_i,
    output logic            [4:0] p_rob_valid_o, // PRF 中存储的值是有效的
    output rob_entry_data_t [4:0] p_rrdata_o,

    // Part 2: ROB 写端口（P级）
    input logic              [1:0] p_valid_i, // 即发射信号
    // input rob_rid_t          [1:0] p_wrrid_i,
    // input rob_entry_static_t [1:0] p_winfo_i,

    // Part 3: ROB 写端口（CDB）
    input pipeline_cdb_t     [1:0] cdb_i,

    // Part 4: ROB 读端口（C级）
    input  rob_rid_t         [1:0] c_rrrid_i,
    output logic             [1:0] c_rob_valid_o,
    output rob_entry_t       [1:0] c_rob_entry_o,

    // 注意，在后端需要撤销时，由 commit 端口对 ROB 中的指令全部执行一次提交，以恢复重命名表的状态
    // 即 RENAME 模块对后端撤销情况并不知情
    input  logic             [1:0] c_retire_i,
    input  logic                   flush_i
  );

  rob_rid_t          [4:0]      p_rrrid_i;
  rob_rid_t          [1:0][1:0] p_rrrid_int;
  rob_rid_t          [1:0] p_wrrid_i;
  rob_entry_static_t [1:0] p_winfo_i;
  // 参数提取
  assign p_rrrid_i = {p_data_i[0].rreg[2], p_rrrid_int};
  for(genvar p = 0 ; p < 2 ; p += 1) begin
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        assign p_rrrid_int[p][i] = p_data_i[p].rreg[i];
    end
    assign p_wrrid_i[p] = p_ctrl_i[p].wreg.rob_id;
    always_comb begin
        p_winfo_i[p] = '0;
        p_winfo_i[p].di = get_rob_from_p(p_ctrl_i[p].di); // 指令控制信息
        p_winfo_i[p].wreg = p_ctrl_i[p].wreg.arch_id;
        // p_winfo_i[p].rreg = p_data_i[p].rreg[i];        // 重命名后的读寄存器
        p_winfo_i[p].wtier = p_ctrl_i[p].wtier;       // 写寄存器 tier id
        p_winfo_i[p].op_code = p_ctrl_i[p].op_code;     // CSR 控制信息
        p_winfo_i[p].csr_id = p_ctrl_i[p].addr_imm[15:2];
        p_winfo_i[p].pc = p_ctrl_i[p].pc;
        p_winfo_i[p].bpu_predict = p_ctrl_i[p].bpu_predict; // 在 ALU 中仅检查是否跳转，跳转执行由提交级负责
        p_winfo_i[p].excp = p_ctrl_i[p].excp;
    end
  end

  // 定义四张表
  // static 表，双写双读
  rob_entry_static_t [1:0] c_rob_entry_static;
  wired_registers_file_banked # (
                                .DATA_WIDTH($bits(rob_entry_static_t)),
                                .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
                                .R_PORT_COUNT(2),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(0)
                              )
                              rob_static (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i(c_rrrid_i),
                                .rdata_o(c_rob_entry_static),
                                .waddr_i(p_wrrid_i),
                                .we_i(p_valid_i),
                                .wdata_i(p_winfo_i)
                              );

  // Data 表
  rob_entry_data_t [4:0] p_rob_entry_data;
  rob_entry_data_t [1:0] c_rob_entry_data;
  wired_registers_file_banked # (
                                .DATA_WIDTH($bits(rob_entry_data_t)),
                                .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
                                .R_PORT_COUNT(7),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(0)
                              )
                              rob_data (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i({c_rrrid_i, p_rrrid_i}),
                                .rdata_o({c_rob_entry_data, p_rob_entry_data}),
                                .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
                                .we_i({cdb_i[1].valid, cdb_i[0].valid}),
                                .wdata_i({cdb_i[1].wdata, cdb_i[0].wdata})
                              );

  // Dynamic 表
  rob_entry_dynamic_t [1:0] c_rob_entry_dynamic;
  rob_entry_dynamic_t [1:0] cdb_entry_dynamic;
  for(genvar i = 0 ; i < 2 ; i++)
  begin
    assign cdb_entry_dynamic[i].excp = cdb_i[i].excp;
    assign cdb_entry_dynamic[i].need_jump = cdb_i[i].need_jump;
    assign cdb_entry_dynamic[i].target_addr = cdb_i[i].target_addr;
    assign cdb_entry_dynamic[i].wrong_forward = cdb_i[i].wrong_forward;
    assign cdb_entry_dynamic[i].uncached = cdb_i[i].uncached;          // 对于 Uncached 的指令，一定会触发流水线冲刷，重新执行，结果直接写入 ARF，不经过 ROB。
    // assign cdb_entry_dynamic[i].store_buffer = cdb_i[i].store_buffer;      // 提交一条 Store_buffer 中的写请求
    // assign cdb_entry_dynamic[i].store_conditional = cdb_i[i].store_conditional; // 条件写，若未命中，则直接失败并冲刷流水线
  end
  wired_registers_file_banked # (
                                .DATA_WIDTH($bits(rob_entry_dynamic_t)),
                                .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
                                .R_PORT_COUNT(2),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(0)
                              )
                              rob_dynamic (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i(c_rrrid_i),
                                .rdata_o(c_rob_entry_dynamic),
                                .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
                                .we_i({cdb_i[1].valid, cdb_i[0].valid}),
                                .wdata_i(cdb_entry_dynamic)
                              );

  // Valid 表，分两部分
  logic [1:0] c_rob_valid_q;
  logic [4:0] p_rob_valid_pt, p_rob_valid_ct;
  logic [1:0] c_rob_valid_pt, c_rob_valid_ct;
  assign p_rob_valid_o = p_rob_valid_pt ^ p_rob_valid_ct;
  // 不用再等待数据就绪，全部送往提交级
  assign c_rob_valid_o = c_rob_valid_q & (c_rob_valid_pt ^ c_rob_valid_ct);
  logic [1:0] p_rob_last_valid, c_rob_last_valid;
  wired_registers_file_banked # (
                                .DATA_WIDTH(1),
                                .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
                                .R_PORT_COUNT(9),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(0)
                              )
                              rob_valid_p (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i({c_rrrid_i, p_rrrid_i, cdb_i[1].wid, cdb_i[0].wid}),
                                .rdata_o({c_rob_valid_pt, p_rob_valid_pt, p_rob_last_valid}),
                                .waddr_i(p_wrrid_i),
                                .we_i(p_valid_i),
                                .wdata_i(c_rob_last_valid)
                              );
  wired_registers_file_banked # (
                                .DATA_WIDTH(1),
                                .DEPTH(1 << `_WIRED_PARAM_ROB_LEN),
                                .R_PORT_COUNT(9),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(0)
                              )
                              rob_valid_c (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i({c_rrrid_i, p_rrrid_i, p_wrrid_i}),
                                .rdata_o({c_rob_valid_ct, p_rob_valid_ct, c_rob_last_valid}),
                                .waddr_i({cdb_i[1].wid, cdb_i[0].wid}),
                                .we_i({cdb_i[1].valid, cdb_i[0].valid}),
                                .wdata_i(~p_rob_last_valid)
                              );

  logic[`_WIRED_PARAM_ROB_LEN:0] valid_rob_entry_q;
  logic[`_WIRED_PARAM_ROB_LEN:0] valid_rob_entry_diff, valid_rob_entry_next;
  assign valid_rob_entry_diff = p_valid_i[1] + p_valid_i[0] - c_retire_i[1] - c_retire_i[0];
  assign valid_rob_entry_next = valid_rob_entry_diff + valid_rob_entry_q;

  always_ff @(posedge clk)
  begin
    if(~rst_n || flush_i) begin
      valid_rob_entry_q <= '0;
      c_rob_valid_q <= '0;
    end else begin
      valid_rob_entry_q <= valid_rob_entry_next;
      c_rob_valid_q[0]  <= valid_rob_entry_next >= 1;
      c_rob_valid_q[1]  <= valid_rob_entry_next >= 2;
    end
  end

  // Output
  assign p_rrdata_o = p_rob_entry_data;
  assign c_rob_entry_o = {
    gather_rob(c_rob_entry_static[1], c_rob_entry_dynamic[1], c_rob_entry_data[1], c_rob_valid_o[1]),
    gather_rob(c_rob_entry_static[0], c_rob_entry_dynamic[0], c_rob_entry_data[0], c_rob_valid_o[0])
                        };

endmodule
