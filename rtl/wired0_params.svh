`ifndef _WIRED_PARAMS_HEADER
`define _WIRED_PARAMS_HEADER

// 定义 Wired 处理器核心规模

// Frontend
`define _WIRED_PARAM_RAS_ADDR_LEN 3
`define _WIRED_PARAM_BHT_DATA_LEN 5

// Backend
// `define _WIRED_PARAM_ENABLE_FPU

`ifdef _WIRED_PARAM_ENABLE_FPU
`define _WIRED_PARAM_PRF_LEN 6
`else
`define _WIRED_PARAM_PRF_LEN 5
`endif //_WIRED_PARAM_ENABLE_FPU

`define _WIRED_PARAM_ROB_LEN 6

`define _WIRED_PARAM_INT_IQ_DEPTH 8
`define _WIRED_PARAM_MDU_IQ_DEPTH 4
`define _WIRED_PARAM_LSU_IQ_DEPTH 4

`define _WIRED_PARAM_INT_IQ_WAKEUP_STACK_SIZE 2

// Performance Related

`define _WIRED_WAKEUP_DST_CACHE_ENABLE // 使能 CACHE 的背靠背唤醒机制

`define _WIRED_STORE_EARLY_ISSUE // 写指令写数据未就绪时，可提前发射

// TLB RELATED
`define _WIRED_PARAM_TLB_CNT 32

`endif
