`include "tl_util.svh"

module wired_mp #(
    parameter int CPU_NUM = 2
  ) (
    input clk,
    input rst_n,
    input [7:0] interrupt_i, // 输入中断
    // 连接到内存总线（AXI-32）
    `AXI_DECLARE_HOST_PORT(32, 32, 4, mem)
  );

  // 转换 TL32 - AXI32
  `TL_DECLARE(32, 32, 4, 1, tl32);
  tl_axi_adapter # (
                   .DataWidth(32),
                   .AddrWidth(32),
                   .SourceWidth(4)
                 )
                 tl_axi_adapter_inst (
                   .clk_i(clk),
                   .rst_ni(rst_n),
                   `TL_CONNECT_DEVICE_PORT(host, tl32),
                   `AXI_FORWARD_HOST_PORT(device, mem)
                 );
  // 转换 TL128 - TL32
  tl_adapter #(
               .HostDataWidth (128),
               .DeviceDataWidth (32),
               .HostAddrWidth (32),
               .DeviceAddrWidth (32),
               .HostSourceWidth (4),
               .DeviceSourceWidth (4),
               .HostSinkWidth (1),
               .DeviceSinkWidth (1),
               .HostMaxSize (4),
               .DeviceMaxSize (4),
               .Fifo (1'b1)
             ) mem_tlul_bridge (
               .clk_i(clk),
               .rst_ni(rst_n),
               `TL_CONNECT_DEVICE_PORT(host, tl128),
               `TL_CONNECT_HOST_PORT(device, tl32)
             );

  // Tilelink Broadcaster - UL
  `TL_DECLARE(128, 32, 4, 1, tl128);
  tl_broadcast #(
                 .DataWidth (128),
                 .AddrWidth (32),
                 .HostSourceWidth (2),
                 .DeviceSourceWidth (4),
                 .SinkWidth (1),
                 .NumCachedHosts (4),
                 .SourceBase ({2'd3, 2'd2, 2'd1, 2'd0}),
                 .SourceMask ({2'd0, 2'd0, 2'd0, 2'd0})
               ) broadcast (
                 .clk_i(clk),
                 .rst_ni(rst_n),
                 `TL_CONNECT_DEVICE_PORT(host, tl_cpu),
                 `TL_CONNECT_HOST_PORT(device, tl128)
               );

  // 核心
  // Tilelink 2-1 转换
  `TL_DECLARE(128, 32, 2, 1, tl_cpu);
  `TL_DECLARE_ARR(128, 32, 2, 1, ch, [1:0]);
  tl_socket_m1 #(
                 .DataWidth(128),
                 .AddrWidth (32),
                 .SourceWidth (2),
                 .SinkWidth (1),
                 .NumLinks (2),

                 .NumSourceRange(1),
                 .SourceBase({2'd2}),
                 .SourceMask({2'd1}),
                 .SourceLink({1'd1})
               ) socket (
                 .clk_i(clk),
                 .rst_ni(rst_n),
                 `TL_CONNECT_DEVICE_PORT(host, ch),
                 `TL_CONNECT_HOST_PORT(device, tl_cpu)
               );
  wired_top #(
              .SOURCE_WIDTH(2),
              .SINK_WIDTH(1),
              .CPU_ID(0),
              .ENABLE_DIFFTEST(0)
            ) cpu_0 (
              `_WIRED_GENERAL_CONN,
              .interrupt_i({1'd0,interrupt_i}),
              `TL_CONNECT_HOST_PORT_IDX(tl, ch, [0])
            );
  wired_top #(
              .SOURCE_WIDTH(2),
              .SINK_WIDTH(1),
              .CPU_ID(1),
              .ENABLE_DIFFTEST(0)
            ) cpu_1 (
              `_WIRED_GENERAL_CONN,
              .interrupt_i({1'd0,'0}),
              `TL_CONNECT_HOST_PORT_IDX(tl, ch, [1])
            );
endmodule
