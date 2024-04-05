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
  logic             c_flush;
  pipeline_cdb_t [1:0]  cdb;
  /* 重命名级 R */
  // 流水线寄存器信号
  logic             r_tid_q;
  always_ff @(posedge clk) begin
    if(!rst_n) begin
      r_tid_q <= '0;
    end else begin
      if(bpu_correct_o.redirect) begin
        r_tid_q <= bpu_correct_o.tid;
      end
    end
  end
  pipeline_ctrl_pack_t[1:0] r_pkg;
  logic r_skid_busy_q;
  assign pkg_ready_o = !r_skid_busy_q;
  wire r_valid = pkg_valid_i | r_skid_busy_q;
  wire r_ready;
  logic [1:0] r_skid_mask_q;
  pipeline_ctrl_pack_t [1:0] r_skid_q;
  always_ff @(posedge clk) begin
    if(!rst_n) begin
        r_skid_busy_q <= 1'b0;
    end else if(c_flush) begin
        if(r_valid && r_pkg[0].bpu_predict.tid == r_tid_q) begin
            r_skid_busy_q <= 1'b1;
        end else begin
            r_skid_busy_q <= 1'b0;
        end
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
  logic [1:0]       l_we;
  logic [1:0]       l_retire;
  arch_rid_t [1:0]  l_waddr;
  rob_rid_t  [1:0]  l_wrrid;
  logic      [1:0]  l_tier_id;
  logic [1:0][31:0] l_wdata;
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
                                .waddr_i(l_waddr),
                                .we_i(l_we & {{(|l_waddr[1])}, {(|l_waddr[0])}}),
                                .wdata_i(l_wdata)
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
                 .c_retire_i(l_retire),
                 .c_warid_i(l_waddr),
                 .c_wrrid_i(l_wrrid),
                 .c_tier_id_i(l_tier_id),
                 .c_flush_i(c_flush),
                 .empty_o(r_empty)
               );
  // 打包生成 P 级需要的包 pipeline_ctrl_p_t
  pipeline_ctrl_p_t [1:0] r_p_pkg;
  pipeline_data_t [1:0] r_p_data;
  for(genvar p = 0 ; p < 2 ; p += 1) begin
    wire [25:0] raw_imm  = r_pkg[p].di.inst[25:0];
    wire [31:0] data_imm = mkimm_data(r_pkg[p].di.imm_type, raw_imm);
    wire [27:0] addr_imm = mkimm_addr(r_pkg[p].di.addr_imm_type, raw_imm);
    always_comb begin
        r_p_data[p] = '0;
        r_p_pkg[p] = '0;
        r_p_data[p].valid = r_arf_valid[p];
        r_p_data[p].rreg = r_rrrid[p];
        for(integer i = 0 ; i < 2 ; i += 1) begin
          if(p == 1 && r_raddr[1][i] == r_waddr[0] && r_waddr[0] != '0) begin
            r_p_data[p].rreg[i] = r_wrrid[0]; // 内部依赖
            r_p_pkg[p].scyc_raw[i] = '1;
          end
        end
        r_p_data[p].rdata[0] = r_pkg[p].di.reg_type_r0 == `_REG_R0_IMM ? data_imm : r_rdata[p][0];
        r_p_data[p].rdata[1] = r_rdata[p][1];
        r_p_pkg[p].di   = get_p_from_d(r_pkg[p].di);
        r_p_pkg[p].wreg.arch_id = r_waddr[p];
        r_p_pkg[p].wreg.rob_id = r_wrrid[p];
        r_p_pkg[p].wtier = r_tier_id[p];
        r_p_pkg[p].addr_imm = addr_imm;
        // if(r_pkg[p].di.invtlb_en || r_pkg[p].di.mem_cacop) begin
        r_p_pkg[p].op_code = raw_imm[4:0]; // invtlb / cacop
        // end else 
        if(r_pkg[p].di.csr_op_en || (r_pkg[p].di.csr_rdcnt != '0)) begin
            r_p_pkg[p].op_code = raw_imm[9:5]; // rj
        end else begin
            r_p_pkg[p].op_code[4] = r_waddr[p] == 5'd1;
            r_p_pkg[p].op_code[3] = r_raddr[p][1] == 5'd1;
        end
        r_p_pkg[p].pc = r_pkg[p].pc;
        r_p_pkg[p].bpu_predict = r_pkg[p].bpu_predict;
        r_p_pkg[p].excp.adef = r_pkg[p].fetch_excp.adef;
        r_p_pkg[p].excp.tlbr = r_pkg[p].fetch_excp.tlbr;
        r_p_pkg[p].excp.pif  = r_pkg[p].fetch_excp.pif;
        r_p_pkg[p].excp.ppi  = r_pkg[p].fetch_excp.ppi;
        r_p_pkg[p].excp.ine  = r_pkg[p].ine;
        r_p_pkg[p].excp.ipe  = r_pkg[p].di.priv_inst && (csr_o.crmd[`_CRMD_PLV] == 2'd3);
        r_p_pkg[p].excp.sys  = r_pkg[p].di.syscall_inst;
        r_p_pkg[p].excp.brk  = r_pkg[p].di.break_inst;
    end
  end
  /* 分发级 P */
  wire alu_ready;
  wire lsu_ready;
  wire mul_ready;
  wire div_ready;
  wire [1:0] p_issue; // 是否有指令被提交
  pipeline_ctrl_p_t [1:0] p_pkg_q;
  pipeline_data_t [1:0] p_data, p_data_q;
  logic [1:0] p_valid_mask_q;
  assign p_issue = p_valid_mask_q & {r_p_ready, r_p_ready};
  assign r_p_ready = alu_ready & lsu_ready & mul_ready & div_ready;
  always_ff @(posedge clk) begin
    if(!rst_n) begin
      p_valid_mask_q <= '0;
    end else begin
      // 注意，这里标识为 valid 的指令，一定已经被重命名了
      // 一定不能在这里阻止其写入 ROB，它需要完整走完提交流水线来回滚重命名表状态
      if(r_p_ready) begin
        p_valid_mask_q <= r_issue;
      end
    end
  end
  always_ff @(posedge clk) begin
    if(r_p_ready) begin
      p_pkg_q  <= r_p_pkg;
      p_data_q <= r_p_data;
    end else begin
      p_data_q <= p_data;
    end
  end
  // CDB 前递
  logic [1:0][1:0]       p_rob_valid;
  logic [1:0][1:0][31:0] p_rob_data;
  for(genvar p = 0 ; p < 2 ; p += 1) begin
    always_comb begin
      p_data[p] = p_data_q[p];
      for(integer i = 0 ; i < 2 ; i += 1) begin
        p_data[p].valid[i] |= p_rob_valid[p][i];
        if(!p_data_q[p].valid[i] && p_rob_valid[p][i]) begin
          p_data[p].rdata[i] = p_rob_data[p][i];
        end
        if(!p_data_q[p].valid[i] &&
            cdb[p_data_q[p].rreg[i][0]].valid &&
            cdb[p_data_q[p].rreg[i][0]].wid[`_WIRED_PARAM_ROB_LEN-1:1] == p_data_q[p].rreg[i][`_WIRED_PARAM_ROB_LEN-1:1]) begin
          p_data[p].rdata[i] = cdb[p_data_q[p].rreg[i][0]].wdata;
          p_data[p].valid[i] = '1;
        end
        p_data[p].valid[i] &= !p_pkg_q[p].scyc_raw[i];
      end
    end
  end
  // ROB (分发 / 提交级别)
  rob_rid_t   [1:0] c_rrrid;
  logic       [1:0] c_rob_valid;
  rob_entry_t [1:0] c_rob_entry;
  logic [1:0]       c_retire;
  wired_rob  wired_rob_inst (
    `_WIRED_GENERAL_CONN,
    // .p_rrrid_i(),
    .p_ctrl_i(p_pkg_q),
    .p_data_i(p_data_q),
    .p_rob_valid_o(p_rob_valid),
    .p_rrdata_o(p_rob_data),
    .p_valid_i(p_issue),
    // .p_wrrid_i(),
    // .p_winfo_i(),

    .cdb_i(cdb),
    .c_rrrid_i(c_rrrid),
    .c_rob_valid_o(c_rob_valid),
    .c_rob_entry_o(c_rob_entry),
    .c_retire_i(c_retire),
    .flush_i(c_flush)
  );

  // IQ（分发 / ROB 写回）
  // CDB 仲裁信号
  localparam CDB_MAX_COUNT = 5;
  logic [CDB_MAX_COUNT-1:0] raw_cdb_ready;
  pipeline_cdb_t [CDB_MAX_COUNT-1:0]  raw_cdb;
  wire [1:0] ifet_excp = {(|p_pkg_q[1].excp), (|p_pkg_q[0].excp)};
  wire [1:0] lsu_valid = p_issue & {p_pkg_q[1].di.lsu_inst,p_pkg_q[0].di.lsu_inst} & ~ifet_excp;
  wire [1:0] div_valid = p_issue & {p_pkg_q[1].di.div_inst,p_pkg_q[0].di.div_inst} & ~ifet_excp;
  wire [1:0] mul_valid = p_issue & {p_pkg_q[1].di.mul_inst,p_pkg_q[0].di.mul_inst} & ~ifet_excp;
  wire [1:0] alu_valid = p_issue & ({p_pkg_q[1].di.alu_inst,p_pkg_q[0].di.alu_inst} | ifet_excp); // 取指阶段存在异常的指令全部送入 ALU
  wired_alu_iq wired_alu_iq_inst (
    `_WIRED_GENERAL_CONN,
    .p_ctrl_i(p_pkg_q),
    .p_data_i(p_data),
    .p_valid_i(alu_valid),
    .p_ready_o(alu_ready),
    .cdb_o(raw_cdb[1:0]),
    .cdb_ready_i(raw_cdb_ready[1:0]),
    .cdb_i(cdb),
    .flush_i(c_flush)
  );
  commit_lsu_req_t   c_lsu_req;
  commit_lsu_resp_t c_lsu_resp;
  logic lsu_iq_valid, lsu_iq_ready, iq_lsu_valid, iq_lsu_ready;
  iq_lsu_req_t  iq_lsu_req;
  iq_lsu_resp_t lsu_iq_resp;
  wired_lsu_iq wired_lsu_iq_inst (
    `_WIRED_GENERAL_CONN,
    .p_ctrl_i(p_pkg_q),
    .p_data_i(p_data),
    .p_valid_i(lsu_valid),
    .p_ready_o(lsu_ready),
    .cdb_o(raw_cdb[2]),
    .cdb_ready_i(raw_cdb_ready[2]),
    .cdb_i(cdb),
    .flush_i(c_flush),
    .lsu_req_valid_o(iq_lsu_valid),
    .lsu_req_ready_i(iq_lsu_ready),
    .lsu_req_o(iq_lsu_req),
    .lsu_resp_valid_i(lsu_iq_valid),
    .lsu_resp_ready_o(lsu_iq_ready),
    .lsu_resp_i(lsu_iq_resp)
  );
  // LSU 例化
  lsu_bus_req_t bus_req;
  lsu_bus_resp_t bus_resp;
  logic [11:0]               p_addr;
  logic [3:0][63:0]         p_rdata;
  cache_tag_t [3:0]          p_rtag;
  logic [1:0]  m_way;
  logic [11:0] m_addr;
  logic [3:0][3:0] m_wstrb;
  logic [3:0][31:0] m_wdata;
  logic [3:0][31:0] m_rdata;
  logic [11:4] t_addr;
  logic  [3:0] t_we;
  cache_tag_t  t_wtag;
  cache_tag_t  [3:0] t_rtag;
  dsram_snoop_t snoop;
  wired_cache_sram # (
             .WORD_SIZE(32)
           )
           wired_cache_sram_inst (
             `_WIRED_GENERAL_CONN,
             .p_addr_i(p_addr),
             .p_rdata_o(p_rdata),
             .p_rtag_o(p_rtag),
             .m_way_i(m_way),
             .m_addr_i(m_addr),
             .m_wstrb_i(m_wstrb),
             .m_wdata_i(m_wdata),
             .m_rdata_o(m_rdata),
             .t_addr_i(t_addr),
             .t_we_i(t_we),
             .t_wtag_i(t_wtag),
             .t_rtag_o(t_rtag)
           );
  wired_lsu  wired_lsu_inst (
          `_WIRED_GENERAL_CONN,
          .lsu_req_valid_i(iq_lsu_valid),
          .lsu_req_ready_o(iq_lsu_ready),
          .lsu_req_i(iq_lsu_req),
          .lsu_resp_valid_o(lsu_iq_valid),
          .lsu_resp_ready_i(lsu_iq_ready),
          .lsu_resp_o(lsu_iq_resp),
          .c_lsu_req_i(c_lsu_req),
          .c_lsu_resp_o(c_lsu_resp),
          .bus_req_o(bus_req),
          .bus_resp_i(bus_resp),
          .snoop_i(snoop),
          .csr_i(csr_o),
          .tlb_update_i(tlb_update_o),
          .p_addr_o(p_addr),
          .p_rdata_i(p_rdata),
          .p_tag_i(p_rtag),
          .flush_i(c_flush)
      );
  wired_tl_adapter # (
        .SOURCE_WIDTH(SOURCE_WIDTH),
        .SINK_WIDTH(SINK_WIDTH),
        .SOURCE_BASE(2 * CPU_ID)
      )
      wired_tl_adapter_inst (
        `_WIRED_GENERAL_CONN,
        .bus_req_i(bus_req),
        .bus_resp_o(bus_resp),
        .snoop_o(snoop),
        .m_way_o(m_way),
        .m_addr_o(m_addr),
        .m_wstrb_o(m_wstrb),
        .m_wdata_o(m_wdata),
        .m_rdata_i(m_rdata),
        .t_addr_o(t_addr),
        .t_we_o(t_we),
        .t_wtag_o(t_wtag),
        .t_rtag_i(t_rtag),
        `TL_FORWARD_HOST_PORT(tl, tl)
      );
  // MDU 例化
  logic ex_mul_req_valid, ex_mul_req_ready, ex_mul_resp_valid, ex_mul_resp_ready;
  logic ex_div_req_valid, ex_div_req_ready, ex_div_resp_valid, ex_div_resp_ready;
  iq_mdu_req_t ex_mul_req, ex_div_req;
  iq_mdu_resp_t ex_mul_resp, ex_div_resp;
  wired_mdu_iq wired_mul_iq_inst (
    `_WIRED_GENERAL_CONN,
    .p_ctrl_i(p_pkg_q),
    .p_data_i(p_data),
    .p_valid_i(mul_valid),
    .p_ready_o(mul_ready),
    .cdb_o(raw_cdb[3]),
    .cdb_ready_i(raw_cdb_ready[3]),
    .cdb_i(cdb),
    .flush_i(c_flush),
    .ex_valid_o(ex_mul_req_valid),
    .ex_ready_i(ex_mul_req_ready),
    .ex_req_o(ex_mul_req),
    .ex_valid_i(ex_mul_resp_valid),
    .ex_ready_o(ex_mul_resp_ready),
    .ex_resp_i(ex_mul_resp)
  );
  wired_mdu_iq wired_div_iq_inst (
    `_WIRED_GENERAL_CONN,
    .p_ctrl_i(p_pkg_q),
    .p_data_i(p_data),
    .p_valid_i(div_valid),
    .p_ready_o(div_ready),
    .cdb_o(raw_cdb[4]),
    .cdb_ready_i(raw_cdb_ready[4]),
    .cdb_i(cdb),
    .flush_i(c_flush),
    .ex_valid_o(ex_div_req_valid),
    .ex_ready_i(ex_div_req_ready),
    .ex_req_o(ex_div_req),
    .ex_valid_i(ex_div_resp_valid),
    .ex_ready_o(ex_div_resp_ready),
    .ex_resp_i(ex_div_resp)
  );
  wired_ex_muler wired_ex_muler_inst (
    `_WIRED_GENERAL_CONN,
    .flush_i(c_flush),
    .valid_i(ex_mul_req_valid),
    .ready_o(ex_mul_req_ready),
    .req_i(ex_mul_req),
    .ready_i(ex_mul_resp_ready),
    .valid_o(ex_mul_resp_valid),
    .resp_o(ex_mul_resp)
  );
  wired_ex_divider wired_ex_divider_inst (
    `_WIRED_GENERAL_CONN,
    .flush_i(c_flush),
    .valid_i(ex_div_req_valid),
    .ready_o(ex_div_req_ready),
    .req_i(ex_div_req),
    .ready_i(ex_div_resp_ready),
    .valid_o(ex_div_resp_valid),
    .resp_o(ex_div_resp)
  );

  // CDB ARBITER
  wired_cdb_arb # (
    .CDB_PORT_CNT(CDB_MAX_COUNT)
  )
  wired_cdb_arb_inst (
    `_WIRED_GENERAL_CONN,
    .cdb_i(raw_cdb),
    .ready_o(raw_cdb_ready),
    .cdb_o(cdb)
  );

  // COMMIT 流水线
  wired_commit  wired_commit_inst (
    `_WIRED_GENERAL_CONN,
    .c_rrrid_o(c_rrrid),
    .c_rob_valid_i(c_rob_valid),
    .c_rob_entry_i(c_rob_entry),
    .c_retire_o(c_retire),
    .c_lsu_req_o(c_lsu_req),
    .c_lsu_resp_i(c_lsu_resp),
    .l_retire_o(l_retire),
    .l_commit_o(l_we),
    .l_data_o(l_wdata),
    .l_warid_o(l_waddr),
    .l_wrrid_o(l_wrrid),
    .l_tier_id_o(l_tier_id),
    .f_upd_o(bpu_correct_o),
    .f_interrupt_i(interrupt_i),
    .l_flush_o(c_flush),
    .rename_empty_i(r_empty),
    .csr_o(csr_o),
    .tlb_update_req_o(tlb_update_o)
  );

endmodule
