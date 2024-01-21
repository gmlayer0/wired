`ifndef _WIRED_STRUCTURE_HEADER
`define _WIRED_STRUCTURE_HEADER

// 此文件中声明了 WIRED 工程将要使用到的全部 structure

`include "decoder.svh"

typedef logic[5:0] arch_rid_t; // 架构寄存器号
typedef logic[6:0] rob_rid_t;  // 重命名后寄存器号 == {}

`endif
