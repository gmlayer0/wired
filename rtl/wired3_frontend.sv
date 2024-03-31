`include "wired0_defines"

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
    input  tlb_update_t         tlb_update_i,
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
                 .p_ready_i(w_skid_ready),
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
            static_excp_t          excp;
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
  wired_icache # (
                 .PACKED_SIZE(2 * $bits(bpu_predict_t))
               )
               wired_icache_inst (
                 `_WIRED_GENERAL_CONN,
                 .f_valid_i(skid_f_valid),
                 .f_ready_o(skid_f_ready),
                 .f_mask_i(w_f.mask),
                 .f_pc_i(w_f.pc),
                 .f_pkg_i(w_f.predict),
                 //  .c_valid_i(),
                 //  .c_ready_o(),
                 //  .c_addr_i(),
                 //  .c_parm_i(),
                 .f_valid_o(f_skid_valid),
                 .f_ready_i(f_skid_ready),
                 .f_mask_o(f_raw.mask),
                 .f_pc_o(f_raw.pc),
                 .f_inst_o(f_raw.inst),
                 .f_pkg_o(f_raw.predict),
                 .f_excp_o(f_raw.excp)
                 .bus_req_o(bus_req),
                 .bus_resp_i(bus_resp),
                 .snoop_i(),
                 .csr_i(csr_i),
                 .tlb_update_i(tlb_update_i),
                 .p_addr_o(),
                 .p_rdata_i(),
                 .p_tag_i(),
                 .p_sll_i(),
                 .flush_i()
               );
  wired_tl_adapter # (
                     .SOURCE_WIDTH(SOURCE_WIDTH),
                     .SINK_WIDTH(SINK_WIDTH),
                     .SOURCE_WIDTH(SOURCE_WIDTH)
                   )
                   wired_tl_adapter_inst (
                     `_WIRED_GENERAL_CONN,
                     .bus_req_i(bus_req),
                     .bus_resp_o(bus_resp),
                     .snoop_i(),
                     .m_way_o(),
                     .m_addr_o(),
                     .m_wstrb_o(),
                     .m_wdata_o(),
                     .m_rdata_i(),
                     .t_addr_o(),
                     .t_we_o(),
                     .t_wtag_o(),
                     .t_rtag_i(),
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
    wired_decoder decoder(
                    .inst_i(d_q.inst[i]),
                    .decode_err_o(d_skid.p[i].ine),
                    .is_o(d_skid.p[i].di)
                  );
    // TODO 生成 register info : ri
    assign d_skid.p[i].pc = {d_q.pc[31:3], (i==0 ? d_q.pc[2] : 1'd1), d_q.pc[1:0]};
    assign d_skid.p[i].bpu_predict = d_q.predict[i];
    assign d_skid.p[i].excp = d_q.excp;
  end

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
        d_skid_ready <= !(d_skid_valid & !skid_b_ready);
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
  logic b_ready;
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
      if(b_ready)
      begin
        b_valid_q <= (b_q.p[0].bpu_predict.tid == b_tid_q) && skid_b_valid; // 丢弃无用包
      end
    end
  end
  d_b_t b_q;
  always_ff @(posedge)
  begin
    if(skid_b_ready)
      b_q <= d_b;
  end

  // MMIO
  `_WIRED_HANDSHAKE_DEFINE(fifo_in, d_b_t);
  `_WIRED_HANDSHAKE_DEFINE(fifo_out, d_b_t);
  d_b_t fifo_raw;
  logic [1:0] fifo_mask;
  always_comb
  begin
    fifo_in_payload = fifo_raw;
    fifo_in_payload.mask = fifo_mask;
  end
  wired_packer # (
                 .CPU_ID(CPU_ID),
                 .PKG_SIZE(PKG_SIZE)
               )
               wired_packer_inst (
                 ._WIRED_GENERAL_DEFINE(_WIRED_GENERAL_DEFINE),
                 .flush_i(g_flush),
                 .valid_i(b_valid_q),
                 .ready_o(b_ready),
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
             )(
               .clk(clk),
               .rst_n(rst_n && !g_flush),
               `_WIRED_INPORT_CONN(inport, fifo_in),
               `_WIRED_OUTPORT_CONN(outport, fifo_out),
             );
  assign pkg_o = fifo_out.p;
  assign pkg_mask_o = fifo_out.mask;
  assign pkg_valid_o = fifo_out_valid;
  assign fifo_out_ready = pkg_ready_i;

endmodule
