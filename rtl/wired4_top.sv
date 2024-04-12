// 此文件是核心的顶层
`include "wired0_defines.svh"

module wired_top #(
    parameter int unsigned SOURCE_WIDTH  = 2,
    parameter int unsigned SINK_WIDTH    = 1,
    parameter int CPU_ID = 0,
    parameter int ENABLE_DIFFTEST = 1
  )(
    `_WIRED_GENERAL_DEFINE,
    input [8:0] interrupt_i, // 输入中断，最高位为 IPI 中断
    // 连接到内存总线（TILELINK-C）
    `TL_DECLARE_HOST_PORT(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl) // tl_a_o

  );
  `TL_DECLARE(128, 32, SOURCE_WIDTH, SINK_WIDTH, tl);
  // FULL REGSLICE
  tl_regslice #(
    .DataWidth (HostDataWidth),
    .AddrWidth (AddrWidth),
    .SourceWidth (SourceWidth),
    .SinkWidth (SinkWidth),
    .RequestMode (2),
    .ReleaseMode (2)
  ) host_reg (
    .clk_i,
    .rst_ni,
    `TL_CONNECT_DEVICE_PORT(host, tl),
    `TL_FORWARD_HOST_PORT(device, tl)
  );

  // Tilelink 2-1 转换
  `TL_DECLARE_ARR(128, 32, SOURCE_WIDTH, SINK_WIDTH, ch, [1:0]);
  tl_socket_m1 #(
                 .DataWidth(128),
                 .AddrWidth (32),
                 .SourceWidth (SOURCE_WIDTH),
                 .SinkWidth (SINK_WIDTH),
                 .NumLinks (2),

                 .NumSourceRange(1),
                 .SourceBase(1'd1),
                 .SourceMask('1 - 1),
                 .SourceLink(1'd1)
               ) socket (
                 .clk_i(clk),
                 .rst_ni(rst_n),
                 `TL_CONNECT_DEVICE_PORT(host, ch),
                 `TL_CONNECT_HOST_PORT(device, tl)
               );

  // 前后端握手信号包
    logic                 pkg_valid;
    logic                 pkg_ready;
    logic [1:0]            pkg_mask;
    pipeline_ctrl_pack_t [1:0]  pkg;
    csr_t                       csr;
    tlb_update_req_t         tlb_update;
    bpu_correct_t       bpu_correct;

  // 生成前端
  wired_frontend # (
                   .SOURCE_WIDTH(SOURCE_WIDTH),
                   .SINK_WIDTH(SINK_WIDTH),
                   .CPU_ID(CPU_ID)
                 )
                 wired_frontend_inst (
                   `_WIRED_GENERAL_CONN,
                   .pkg_valid_o(pkg_valid),
                   .pkg_ready_i(pkg_ready),
                   .pkg_mask_o(pkg_mask),
                   .pkg_o(pkg),
                   .csr_i(csr),
                   .tlb_update_i(tlb_update),
                   .bpu_correct_i(bpu_correct),
                   `TL_CONNECT_HOST_PORT_IDX(tl, ch, [1])
                 );

  // 生成后端
  wired_backend # (
                  .SOURCE_WIDTH(SOURCE_WIDTH),
                  .SINK_WIDTH(SINK_WIDTH),
                  .CPU_ID(CPU_ID),
                  .ENABLE_DIFFTEST(ENABLE_DIFFTEST)
                )
                wired_backend_inst (
                  `_WIRED_GENERAL_CONN,
                  .interrupt_i(interrupt_i),
                  .pkg_valid_i(pkg_valid),
                  .pkg_ready_o(pkg_ready),
                  .pkg_mask_i(pkg_mask),
                  .pkg_i(pkg),
                  .csr_o(csr),
                  .tlb_update_o(tlb_update),
                  .bpu_correct_o(bpu_correct),
                  `TL_CONNECT_HOST_PORT_IDX(tl, ch, [0])
                );


endmodule
