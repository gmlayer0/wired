`include "wired0_defines.svh"

function reg_info_t get_register_info(
    input decode_info_d_t di,
    input logic[31:0] inst
  );
  reg_info_t ret;

  logic [2:0] r0_sel, r1_sel;
  logic [1:0] w_sel;
  r0_sel = di.reg_type_r0;
  r1_sel = di.reg_type_r1;
  w_sel  = di.reg_type_w;
  case(r0_sel)
    default :
    begin
      ret.r_reg[0] = '0;
    end
    `_REG_RD :
    begin
      ret.r_reg[0] = inst[4:0];
    end
    `_REG_RJ :
    begin
      ret.r_reg[0] = inst[9:5];
    end
    `_REG_RK :
    begin
      ret.r_reg[0] = inst[14:10];
    end
  endcase
  case(r1_sel)
    default :
    begin
      ret.r_reg[1] = '0;
    end
    `_REG_RD :
    begin
      ret.r_reg[1] = inst[4:0];
    end
    `_REG_RJ :
    begin
      ret.r_reg[1] = inst[9:5];
    end
    `_REG_RK :
    begin
      ret.r_reg[1] = inst[14:10];
    end
  endcase
  case(w_sel)
    default :
    begin
      ret.w_reg = '0;
    end
    `_REG_W_RD :
    begin
      ret.w_reg = inst[4:0];
    end
    `_REG_W_RJD :
    begin
      ret.w_reg = inst[4:0] | inst[9:5];
    end
    `_REG_W_BL1 :
    begin
      ret.w_reg = 5'd1;
    end
  endcase
  return ret;
endfunction

// Fuction module for Wired project
// Frontend module
module wired_frontend #(
    parameter int unsigned SOURCE_WIDTH  = 1,
    parameter int unsigned SINK_WIDTH    = 1,
    parameter int CPU_ID = 0
  )(
    `_WIRED_GENERAL_DEFINE,

    // 连接到后端
    output logic                 pkg_valid_o,
    input  logic                 pkg_ready_i,
    output logic [1:0]            pkg_mask_o,
    output pipeline_ctrl_pack_t [1:0]  pkg_o,

    // 后端反馈
    input  csr_t                       csr_i,
    input  tlb_update_req_t         tlb_update_i,
    input  bpu_correct_t       bpu_correct_i,

    // 连接到内存总线（TILELINK-C）
    `TL_DECLARE_HOST_PORT(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl) // tl_a_o
  );

  wire g_flush = bpu_correct_i.redirect; // 前端刷新信号

  // PC 生成部分结构体及数据定义
  typedef struct packed {
            logic        [31:0]      pc;
            logic         [1:0]    mask;
            bpu_predict_t [1:0] predict;
          } w_t;
  w_t w_pcgen;
  logic w_pcgen_valid, w_pcgen_ready;

  wired_pcgen  wired_pcgen_inst (
                 `_WIRED_GENERAL_CONN,
                 .p_correct_i(bpu_correct_i),
                 .p_ready_i(w_pcgen_ready || g_flush),
                 .p_valid_o(w_pcgen_valid),
                 .p_pc_o(w_pcgen.pc),
                 .p_mask_o(w_pcgen.mask),
                 .p_predict_o(w_pcgen.predict)
               );

  // icache 握手
  logic f_req_valid, f_req_ready;
  w_t f_pcgen;
  wired_skidbuf #(
    .T(w_t)
  ) wired_wf_skidbuf_inst (
    .clk(clk),
    .rst_n(rst_n && !g_flush),
    .inp_valid_i(w_pcgen_valid),
    .inp_ready_o(w_pcgen_ready),
    .inp_i(w_pcgen),
    .oup_valid_o(f_req_valid),
    .oup_ready_i(f_req_ready),
    .oup_o(f_pcgen)
  );

  // ICache 生成部分结构体定义
  typedef struct packed {
            logic        [31:0]      pc;
            logic   [1:0][31:0]    inst;
            logic         [1:0]    mask;
            bpu_predict_t [1:0] predict;
            fetch_excp_t           excp;
          } f_t;
  f_t f_icache;
  logic f_resp_valid, f_resp_ready;
  f_t skid_d;
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
                     .WORD_SIZE(64)
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
  iq_lsu_req_t  f_icache_req;
  iq_lsu_resp_t f_icache_resp;
  logic[2 * $bits(bpu_predict_t) + 1:0] f_pkg;
  always_comb begin
    f_icache_req = '0;
    f_icache_req.cacop = RD_ALLOC;
    f_icache_req.msize = &f_pcgen.mask ? 2'd3 : 2'd2; // 64 Bits / 32 Bits
    f_icache_req.vaddr = f_pcgen.pc;
    f_icache.pc = f_icache_resp.vaddr;
    f_icache.inst = f_icache_resp.rdata;
    f_icache.excp = f_icache_resp.f_excp;
    {f_icache.predict, f_icache.mask} = f_pkg;
    if(!f_icache.mask[0]) begin
      f_icache.mask = {1'b0, f_icache.mask[1]};
      f_icache.inst[0] = f_icache.inst[1];
      f_icache.predict[0] = f_icache.predict[1];
    end
  end
  wired_cache #(
                .ICACHE(1),     // 配置是否为 ICache
                .OUTPUT_BUF(0), // 状态机输出到 lsu_resp 不用再打一拍
                .SRAM_WIDTH(64),
                .PKG_SIZE(2 * $bits(bpu_predict_t) + 2)
              )
              wired_icache_inst (
                `_WIRED_GENERAL_CONN,
                .lsu_req_valid_i(f_req_valid),
                .lsu_req_ready_o(f_req_ready),
                .lsu_req_i(f_icache_req),
                .lsu_pkg_i({f_pcgen.predict, f_pcgen.mask}),
                .lsu_resp_valid_o(f_resp_valid),
                .lsu_resp_ready_i(f_resp_ready),
                .lsu_resp_o(f_icache_resp),
                .lsu_pkg_o(f_pkg),
                .c_lsu_req_i('0),
                // .c_lsu_resp_o/(/**/),
                .bus_req_o(bus_req),
                .bus_resp_i(bus_resp),
                .snoop_i(snoop),
                .csr_i(csr_i),
                .tlb_update_i(tlb_update_i),
                .p_addr_o(p_addr),
                .p_rdata_i(p_rdata),
                .p_tag_i(p_rtag),
                .flush_i(g_flush)
              );
  wired_tl_adapter # (
                     .SOURCE_WIDTH(SOURCE_WIDTH),
                     .SINK_WIDTH(SINK_WIDTH),
                     .SOURCE_BASE(2 * CPU_ID + 1)
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

  // D 级握手处理
  f_t d_icache;
  logic d_valid, d_ready;
  // 进行 TID 过滤
  logic d_tid_q;
  always_ff @(posedge clk) begin
    if(!rst_n) d_tid_q <= '0;
    else if(g_flush) d_tid_q <= bpu_correct_i.tid;
  end
  wired_fifo #(
    .DATA_WIDTH($bits(f_t)),
    .DEPTH(2)
  ) wired_fd_pipe_inst (
    .clk(clk),
    .rst_n(rst_n && !g_flush),
    .inport_valid_i(f_resp_valid),
    .inport_ready_o(f_resp_ready),
    .inport_payload_i(f_icache),
    .outport_valid_o(d_valid),
    .outport_ready_i(d_ready/* || (d_icache.predict[0].tid != d_tid_q)*/), // 开始过滤
    .outport_payload_o(d_icache)
  );
  // Align(already) Decode stage
  typedef struct packed {
            pipeline_ctrl_pack_t [1:0]    p;
            logic                [1:0] mask;
          } d_t;
  d_t d_decode;

  for(genvar i = 0 ; i < 2 ; i++)
  begin : gen_decoder
    wire decode_err;
    decode_info_d_t di;
    wired_decoder decoder(
                    .inst_i(d_icache.inst[i]),
                    .decode_err_o(decode_err),
                    .is_o(di)
                  );
    always_comb begin
      d_decode.p[i].di = di;
      d_decode.p[i].ri = get_register_info(di, d_icache.inst[i]);
      d_decode.p[i].pc = {d_icache.pc[31:3], (i==0 ? d_icache.pc[2] : 1'd1), d_icache.pc[1:0]};
      d_decode.p[i].bpu_predict = d_icache.predict[i];
      d_decode.p[i].fetch_excp = d_icache.excp;
      d_decode.p[i].ine = !(|d_icache.excp) &&
      (decode_err || (di.invtlb_en && (d_icache.inst[i][4:0] >= 5'd7)));
    end
  end
  assign d_decode.mask = d_icache.mask;

  d_t b_decode;
  logic b_valid, b_ready;
  wired_pipereg #(
    .T(d_t)
  ) wired_db_pipe_inst (
    .clk(clk),
    .rst_n(rst_n && !g_flush),
    .inp_valid_i(d_valid/* && (d_icache.predict[0].tid == d_tid_q)*/),
    .inp_ready_o(d_ready),
    .inp_i(d_decode),
    .oup_valid_o(b_valid),
    .oup_ready_i(b_ready),
    .oup_o(b_decode)
  );

  // MMIO
  `_WIRED_HANDSHAKE_DEFINE(fifo_in, d_t);
  `_WIRED_HANDSHAKE_DEFINE(fifo_out, d_t);
  pipeline_ctrl_pack_t [1:0] fifo_raw;
  logic [1:0] fifo_mask;
  wired_packer # (
                 .PKG_SIZE($bits(pipeline_ctrl_pack_t))
               )
               wired_packer_inst (
                 `_WIRED_GENERAL_CONN,
                 .flush_i(g_flush),
                 .valid_i(b_valid),
                 .ready_o(b_ready),
                 .nz_i({(b_decode.p[1].ri.w_reg!='0),(b_decode.p[0].ri.w_reg!='0)}),
                 .bank_i({b_decode.p[1].ri.w_reg[0],b_decode.p[0].ri.w_reg[0]}),
                 .pkg_i(b_decode.p),
                 .mask_i(b_decode.mask),
                 .valid_o(fifo_in_valid),
                 .ready_i(fifo_in_ready),
                 .pkg_o(fifo_raw),
                 .mask_o(fifo_mask)
               );
  always_comb
  begin
    fifo_in_payload.p = fifo_raw;
    fifo_in_payload.mask = fifo_mask;
  end

  // FIFO
  wired_fifo #(
               .DATA_WIDTH($bits(d_t)),
               .DEPTH(8)
             )
             wired_pkg_fifo(
               .clk(clk),
               .rst_n(rst_n && !g_flush),
               `_WIRED_INPORT_CONN(inport, fifo_in),
               `_WIRED_OUTPORT_CONN(outport, fifo_out)
             );
  // 随机暂停，积累足够多指令
  logic [19:0] timer_q;
  always_ff @(posedge clk) if(!rst_n) timer_q <= '0; else timer_q <= timer_q + 20'd1;
  wire rnd_go = /*timer_q[6]*/ '1; // 64 拍暂停，再执行
  assign pkg_o = fifo_out_payload.p;
  assign pkg_mask_o = fifo_out_payload.mask;
  assign pkg_valid_o = fifo_out_valid & rnd_go;
  assign fifo_out_ready = pkg_ready_i & rnd_go;

endmodule
