`ifndef _WIRED_PARAMS_HEADER
`define _WIRED_PARAMS_HEADER

// 定义 Wired 处理器核心规模

// Frontend
`define _WIRED_PARAM_RAS_ADDR_LEN 3
`define _WIRED_PARAM_BHT_HISTORY_LEN 5
`define _WIRED_PARAM_BHT_PC_LEN 8
`define _WIRED_PARAM_BINFO_LEN 9

// Backend
`define _WIRED_PARAM_ENABLE_FPU

`ifdef _WIRED_PARAM_ENABLE_FPU
`define _WIRED_PARAM_ARF_LEN 6
`else
`define _WIRED_PARAM_ARF_LEN 5
`endif //_WIRED_PARAM_ENABLE_FPU

`define _WIRED_PARAM_ROB_LEN 6

`define _WIRED_PARAM_INT_IQ_DEPTH 8
`define _WIRED_PARAM_MDU_IQ_DEPTH 4
`define _WIRED_PARAM_LSU_IQ_DEPTH 4
`define _WIRED_PARAM_FPU_IQ_DEPTH 2

`define _WIRED_PARAM_INT_IQ_WAKEUP_STACK_SIZE 2

// Performance Related

`define _WIRED_WAKEUP_DST_CACHE_ENABLE // 使能 CACHE 的背靠背唤醒机制

`define _WIRED_STORE_EARLY_ISSUE // 写指令写数据未就绪时，可提前发射

`define _WIRED_TDP_ARF // 使用真双口寄存器堆实现 ARF/RENAME ，以实现更高效率的后端流水线

`define _WIRED_UNCACHE_DBAR 1 // 定义 USTORE 指令是否会产生 Barrier 效果

`define _WIRED_USTORE_DEPTH 1 // 注意，对于内存序要求高的场景一定要配置为 0 ，否则有概率卡死

// TLB RELATED
`define _WIRED_PARAM_TLB_CNT 32

`endif
