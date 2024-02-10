`include "wired0_defines"

// Fuction module for Wired project
// This module take two inst register infomation as input(combinational)
module wired_backend #(
    parameter CPU_ID = 1'd0
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到前端（中断由前端输入，打在指令包中）

    // 连接到内存总线（TILELINK-C）

);

    // 解码级 / WAW 冲突解除 D

    // 重命名级 R

    // ROB (分发 / 提交级别)

    // Reserve Station（分发 / ROB 写回）

    // 执行单元

endmodule
