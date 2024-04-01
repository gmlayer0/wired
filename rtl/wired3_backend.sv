`include "wired0_defines.svh"

// Fuction module for Wired project
module wired_backend #(
    parameter int unsigned SOURCE_WIDTH  = 1,
    parameter int unsigned SINK_WIDTH    = 1,
    parameter int CPU_ID = 0
  )(
    `_WIRED_GENERAL_DEFINE,

    input [8:0] interrupt_i, // 输入中断

    // 来自前端（中断由前端输入，打在指令包中）
    input  logic                 pkg_valid_i,
    output logic                 pkg_ready_o,
    input  logic [1:0]            pkg_mask_i,
    input  pipeline_ctrl_pack_t [1:0]  pkg_i,

    // 后端反馈
    output csr_t                       csr_o,
    output tlb_update_req_t         tlb_update_o,
    output bpu_correct_t       bpu_correct_o,

    // 连接到内存总线（TILELINK-C）
    `TL_DECLARE_HOST_PORT(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl)
  );
  /* 重命名级 R */
  // 流水线寄存器信号
  logic r_skid_busy_q;
  assign pkg_ready_o = r_skid_busy_q;
  wire r_valid = pkg_valid_i | r_skid_busy_q;
  wire r_ready;
  logic [1:0] r_skid_mask_q;
  pipeline_ctrl_pack_t [1:0] r_skid_q;
  always_ff @(posedge clk) begin
    if(!rst_n) begin
        r_skid_busy_q <= 1'b0;
    end else begin
        r_skid_busy_q <= !r_ready && r_valid;
    end
  end
  always_ff @(posedge clk) begin
    if(!r_skid_busy_q) begin
        r_skid_mask_q <= pkg_mask_i;
        r_skid_q <= pkg_i;
    end
  end
  logic[1:0] r_mask;
  wire[1:0] r_issue = {r_ready, r_ready} & r_mask;
  pipeline_ctrl_pack_t[1:0] r_pkg;
  assign r_mask = {r_valid, r_valid} & (r_skid_busy_q ? r_skid_mask_q : pkg_mask_i);
  assign r_pkg = r_skid_busy_q ? r_skid_q : pkg_i;
  // --- ARF ---
  // 连接到 r_pkg 中的寄存器号
  arch_rid_t [1:0][1:0] r_raddr;
  rob_rid_t  [1:0][1:0] r_rrrid; // Rename_read_rob_register_id
  logic      [1:0][1:0] r_arf_valid;
  logic[1:0][1:0][31:0] r_rdata;
  arch_rid_t [1:0]      r_waddr;
  rob_rid_t  [1:0]      r_wrrid;
  logic      [1:0]      r_tier_id;
  for(genvar i = 0; i < 2; i++) begin
    for(genvar r = 0; r < 2; r++) begin
      assign r_raddr[i][r] = r_pkg[i].ri.r_reg[r];
    end
    assign r_waddr[i] = r_pkg[i].ri.w_reg;
  end
  // 连接到提交级的信号
  logic [1:0]       c_we;
  logic [1:0]       c_retire;
  arch_rid_t [1:0]  c_waddr;
  rob_rid_t  [1:0]  c_wrrid;
  logic      [1:0]  c_tier_id;
  logic [1:0][31:0] c_wdata;
  logic             c_flush;
  logic             r_empty;
  wired_registers_file_banked # (
                                .DATA_WIDTH(32),
                                .DEPTH(32),
                                .R_PORT_COUNT(4),
                                .W_PORT_COUNT(2),
                                .NEED_RESET(1),
                                .NEED_FORWARD(1)
                              )
                              arf (
                                `_WIRED_GENERAL_CONN,
                                .raddr_i(r_raddr),
                                .rdata_o(r_rdata),
                                .waddr_i(c_waddr),
                                .we_i(c_we & {{(|c_waddr[1])}, {(|c_waddr[0])}}),
                                .wdata_i(c_wdata)
                              );
  // --- RENAME ---
  // 握手信号
  wire r_p_valid, r_p_ready;
  assign r_p_valid = r_valid;
  wired_rename # (
                 .DEPTH(32)
               )
               wired_rename_inst (
                `_WIRED_GENERAL_CONN,
                 .r_rarid_i(r_raddr),
                 .r_rrrid_o(r_rrrid),
                 .r_prf_valid_o(r_arf_valid),
                 .r_mask_i(r_mask),
                 .r_ready_o(r_ready),
                 .r_warid_i(r_waddr),
                 .r_wrrid_o(r_wrrid),
                 .r_tier_id_o(r_tier_id),
                 .p_ready_i(r_p_ready),
                 .c_retire_i(c_retire),
                 .c_warid_i(c_waddr),
                 .c_wrrid_i(c_wrrid),
                 .c_tier_id_i(c_tier_id),
                 .c_flush_i(c_flush),
                 .empty_o(r_empty)
               );
  // 打包生成 P 级需要的包 pipeline_ctrl_p_t
  pipeline_ctrl_p_t [1:0] p_pkg;
  for(genvar p = 0 ; p < 2 ; p += 1) begin
    wire [25:0] raw_imm  = r_pkg[p].inst[25:0];
    wire [31:0] data_imm = mkimm_data(r_pkg[p].di.imm_type, raw_imm);
    wire [27:0] addr_imm = mkimm_addr(r_pkg[p].di.addr_imm_type, raw_imm);
    always_comb begin
        p_pkg[p] = '0;
        p_pkg[p].di   = get_p_from_d(r_pkg[p].di);
        p_pkg[p].wreg.arch_id = r_waddr[p];
        p_pkg[p].wreg.rob_id = r_wrrid[p];
        p_pkg[p].wtire = r_tier_id[p];
        p_pkg[p].addr_imm = addr_imm;
        // if(r_pkg[p].di.invtlb_en || r_pkg[p].di.mem_cacop) begin
        p_pkg[p].op_code = raw_imm[4:0]; // invtlb / cacop
        // end else 
        if(r_pkg[p].di.csr_op_en || r_pkg[p].di.csr_rdcnt) begin
            p_pkg[p].op_code = raw_imm[9:5]; // rj
        end else begin
            p_pkg[p].op_code[4] = r_waddr[p] == 5'd1;
            p_pkg[p].op_code[3] = r_raddr[p][1] == 5'd1;
        end
        p_pkg[p].pc = r_pkg[p].pc;
        p_pkg[p].bpu_predict = r_pkg[p].bpu_predict;
        p_pkg[p].excp = r_pkg[p].fetch_excp;
    end
  end
  /* 分发级 P */
  // ROB (分发 / 提交级别)

  // Reserve Station（分发 / ROB 写回）

  // 执行单元

endmodule
