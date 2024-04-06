`include "wired0_defines.svh"

function reg_info_t get_register_info(
    input decode_info_d_t di,
    input logic[31:0] inst
  );
  reg_info_t ret;

  logic [1:0] r0_sel, w_sel;
  logic r1_sel;
  r0_sel = di.reg_type_r0;
  r1_sel = di.reg_type_r1;
  w_sel  = di.reg_type_w;
  case(r0_sel)
    default :
    begin
      ret.r_reg[0] = '0;
    end
    `_REG_R0_RK :
    begin
      ret.r_reg[0] = inst[14:10];
    end
    `_REG_R0_RD :
    begin
      ret.r_reg[0] = inst[4:0];
    end
  endcase
  case(r1_sel)
    default :
    begin
      ret.r_reg[1] = '0;
    end
    `_REG_R1_RJ :
    begin
      ret.r_reg[1] = inst[9:5];
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

  logic w_skid_valid, w_skid_ready;
  logic skid_f_valid, skid_f_ready;
  assign skid_f_valid = w_skid_valid | !w_skid_ready;
  typedef struct packed {
            logic        [31:0]      pc;
            logic         [1:0]    mask;
            bpu_predict_t [1:0] predict;
          } w_f_t;
  w_f_t w_skid;
  w_f_t skid_f;
  w_f_t w_f;

  wired_pcgen  wired_pcgen_inst (
                 `_WIRED_GENERAL_CONN,
                 .p_correct_i(bpu_correct_i),
                 .p_ready_i(w_skid_ready || g_flush),
                 .p_valid_o(w_skid_valid),
                 .p_pc_o(w_skid.pc),
                 .p_mask_o(w_skid.mask),
                 .p_predict_o(w_skid.predict)
               );

  // w_f skid buf
  if( 1 )
  begin : W_F_SKIDBUF
    always_ff @(posedge clk)
    begin
      if(!rst_n || g_flush)
      begin
        w_skid_ready <= '1;
      end
      else
      begin
        w_skid_ready <= !(w_skid_valid & !skid_f_ready);
      end
    end

    always_ff @(posedge clk)
    begin
      if(w_skid_ready)
      begin
        skid_f <= w_skid;
      end
    end

    assign w_f = w_skid_ready ? w_skid : skid_f;
  end

  // icache
  logic f_skid_valid, f_skid_ready;
  logic skid_d_valid, skid_d_ready;
  assign skid_d_valid = f_skid_valid | !f_skid_ready;
  typedef struct packed {
            logic        [31:0]      pc;
            logic   [1:0][31:0]    inst;
            logic         [1:0]    mask;
            bpu_predict_t [1:0] predict;
            fetch_excp_t           excp;
          } f_d_t;
  f_d_t f_raw, f_skid, f_d;
  always_comb
  begin
    f_skid = f_raw;
    if(!f_raw.mask[0])
    begin
      f_skid.mask = {1'b0, f_raw.mask[1]};
      f_skid.inst[0] = f_raw.inst[1];
      f_skid.predict[0] = f_raw.predict[1];
    end
  end
  f_d_t skid_d;
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
  iq_lsu_req_t  icache_req;
  iq_lsu_resp_t icache_resp;
  always_comb begin
    icache_req = '0;
    icache_req.cacop = RD_ALLOC;
    icache_req.msize = &w_f.mask ? 2'd3 : 2'd2; // 64 Bits / 32 Bits
    icache_req.vaddr = w_f.pc;
    assign f_raw.pc = icache_resp.vaddr;
    assign f_raw.inst = icache_resp.rdata;
    assign f_raw.excp = icache_resp.f_excp;
  end
  wired_cache #(
                .ICACHE(1),     // 配置是否为 ICache
                .OUTPUT_BUF(1), // 状态机输出到 lsu_resp 再打一拍
                .SRAM_WIDTH(64),
                .PKG_SIZE(2 * $bits(bpu_predict_t) + 2)
              )
              wired_icache_inst (
                `_WIRED_GENERAL_CONN,
                .lsu_req_valid_i(skid_f_valid),
                .lsu_req_ready_o(skid_f_ready),
                .lsu_req_i(icache_req),
                .lsu_pkg_i({w_f.predict, w_f.mask}),
                .lsu_resp_valid_o(f_skid_valid),
                .lsu_resp_ready_i(f_skid_ready),
                .lsu_resp_o(icache_resp),
                .lsu_pkg_o({f_raw.predict, f_raw.mask}),
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

  // f_d skid buf
  if( 1 )
  begin : F_D_SKIDBUF
    always_ff @(posedge clk)
    begin
      if(!rst_n || g_flush)
      begin
        f_skid_ready <= '1;
      end
      else
      begin
        f_skid_ready <= !(f_skid_valid & !skid_d_ready);
      end
    end

    always_ff @(posedge clk)
    begin
      if(f_skid_ready)
      begin
        skid_d <= f_skid;
      end
    end

    assign f_d = f_skid_ready ? f_skid : skid_d;
  end

  // Align(already) Decode stage
  typedef struct packed {
            pipeline_ctrl_pack_t [1:0]    p;
            logic                [1:0] mask;
          } d_b_t;
  logic d_skid_ready, d_skid_valid;
  logic skid_b_ready, skid_b_valid;
  assign skid_b_valid = d_skid_valid | !d_skid_ready;
  d_b_t d_skid, d_b;
  d_b_t skid_b;
  logic d_valid_q;
  f_d_t d_q;
  // D 级别握手处理
  always_ff @(posedge clk)
  begin
    if(!rst_n || g_flush)
    begin
      d_valid_q <= '0;
    end
    else
    begin
      if(d_skid_ready)
        d_valid_q <= skid_d_valid;
    end
  end
  always_ff @(posedge clk)
  begin
    if(skid_d_ready)
    begin
      d_q <= f_d;
    end
  end
  assign d_skid_valid = d_valid_q;
  assign skid_d_ready = (!d_valid_q) || d_skid_ready;

  for(genvar i = 0 ; i < 2 ; i++)
  begin : gen_decoder
    wire decode_err;
    wired_decoder decoder(
                    .inst_i(d_q.inst[i]),
                    .decode_err_o(decode_err),
                    .is_o(d_skid.p[i].di)
                  );
    assign d_skid.p[i].ri = get_register_info(d_skid.p[i].di, d_q.inst[i]);
    assign d_skid.p[i].pc = {d_q.pc[31:3], (i==0 ? d_q.pc[2] : 1'd1), d_q.pc[1:0]};
    assign d_skid.p[i].bpu_predict = d_q.predict[i];
    assign d_skid.p[i].fetch_excp = d_q.excp;
    assign d_skid.p[i].ine = ~(|d_q.excp) & decode_err;
  end
  assign d_skid.mask = d_q.mask;

  // d_b skid buf
  if( 1 )
  begin : D_B_SKIDBUF
    always_ff @(posedge clk)
    begin
      if(!rst_n || g_flush)
      begin
        d_skid_ready <= '1;
      end
      else
      begin
        d_skid_ready <= !(skid_b_valid & !skid_b_ready);
      end
    end

    always_ff @(posedge clk)
    begin
      if(d_skid_ready)
      begin
        skid_b <= d_skid;
      end
    end

    assign d_b = d_skid_ready ? d_skid : skid_b;
  end

  // D -> B 握手处理
  logic b_valid_q, b_tid_q;
  always_ff @(posedge clk)
  begin
    if(!rst_n)
    begin
      b_tid_q <= '0;
    end
    else if(bpu_correct_i.redirect)
    begin
      b_tid_q <= bpu_correct_i.tid;
    end
  end
  always_ff @(posedge clk)
  begin
    if(!rst_n || g_flush)
    begin
      b_valid_q <= '0;
    end
    else
    begin
      if(skid_b_ready)
      begin
        b_valid_q <= (d_b.p[0].bpu_predict.tid == b_tid_q) && skid_b_valid; // 丢弃无用包
      end
    end
  end
  d_b_t b_q;
  always_ff @(posedge clk)
  begin
    if(skid_b_ready)
      b_q <= d_b;
  end

  // MMIO
  `_WIRED_HANDSHAKE_DEFINE(fifo_in, d_b_t);
  `_WIRED_HANDSHAKE_DEFINE(fifo_out, d_b_t);
  pipeline_ctrl_pack_t [1:0] fifo_raw;
  logic [1:0] fifo_mask;
  always_comb
  begin
    fifo_in_payload.p = fifo_raw;
    fifo_in_payload.mask = fifo_mask;
  end
  wired_packer # (
                 .PKG_SIZE($bits(pipeline_ctrl_pack_t))
               )
               wired_packer_inst (
                 `_WIRED_GENERAL_CONN,
                 .flush_i(g_flush),
                 .valid_i(b_valid_q),
                 .ready_o(skid_b_ready),
                 .nz_i({(b_q.p[1].ri.w_reg!='0),(b_q.p[0].ri.w_reg!='0)}),
                 .bank_i({b_q.p[1].ri.w_reg[0],b_q.p[0].ri.w_reg[0]}),
                 .pkg_i(b_q.p),
                 .mask_i(b_q.mask),
                 .valid_o(fifo_in_valid),
                 .ready_i(fifo_in_ready),
                 .pkg_o(fifo_raw),
                 .mask_o(fifo_mask)
               );

  // FIFO
  wired_fifo #(
               .DATA_WIDTH($bits(d_b_t)),
               .DEPTH(8)
             )
             wired_pkg_fifo(
               .clk(clk),
               .rst_n(rst_n && !g_flush),
               `_WIRED_INPORT_CONN(inport, fifo_in),
               `_WIRED_OUTPORT_CONN(outport, fifo_out)
             );
  assign pkg_o = fifo_out_payload.p;
  assign pkg_mask_o = fifo_out_payload.mask;
  assign pkg_valid_o = fifo_out_valid;
  assign fifo_out_ready = pkg_ready_i;

endmodule
