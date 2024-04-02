// 此文件是核心的顶层
`include "wired0_defines.svh"

module wired_sp (
    `_WIRED_GENERAL_DEFINE,
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
               .DeviceSourceWidth (1),
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
  `TL_DECLARE(128, 32, 1, 1, tl128);
  tl_broadcast #(
                 .DataWidth (128),
                 .AddrWidth (32),
                 .HostSourceWidth (1),
                 .DeviceSourceWidth (4),
                 .SinkWidth (1),
                 .NumCachedHosts (1),
                 .SourceBase ({4'd1 /*,4'd0*/}),
                 .SourceMask ({4'd14/*,4'd14*/})
               ) broadcast (
                .clk_i(clk),
                .rst_ni(rst_n),
                 `TL_CONNECT_DEVICE_PORT(host, tl_cpu),
                 `TL_CONNECT_HOST_PORT(device, tl128)
               );

  // 核心
  `TL_DECLARE(128, 32, 4, 1, tl_cpu);
  wired_top #(
              .SOURCE_WIDTH(4),
              .SINK_WIDTH(1),
              .CPU_ID(0)
            ) cpu (
              `_WIRED_GENERAL_CONN,
              .interrupt_i({1'd1,interrupt_i}),
              `TL_CONNECT_HOST_PORT(tl, tl_cpu)
            );

endmodule
