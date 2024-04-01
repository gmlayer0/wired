// dcache cpu side
`include "wired0_defines.svh"

// 为 lsu 设计的 TAG Tracker，实时检查对于 Cache 的写情况，并更新 hit 状态
module wired_lsu_hit_snop(
    `_WIRED_GENERAL_DEFINE,

    // 写入端口
    input logic we,
    input logic [31:0] paddr,
    input logic [31:0] wdata,
    input logic  [3:0] wstrb,
    input logic  [3:0] init_hit, // 初始态命中情况

    // 查询端口

    // SRAM Snoop 端口，实时更新所有 sb 表项

);

    
endmodule
