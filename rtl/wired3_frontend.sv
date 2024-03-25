`include "wired0_defines"

// Fuction module for Wired project
// Frontend module
module wired_frontend #(
    parameter CPU_ID = 1'd0
)(
    `_WIRED_GENERAL_DEFINE,

    // 连接到前端（中断由前端输入，打在指令包中）

    // 连接到内存总线（TILELINK-C）

);

endmodule
