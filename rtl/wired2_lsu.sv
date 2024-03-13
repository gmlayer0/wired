// dcache cpu side
`include "wired0_defines"

module wired_lsu(
    `_WIRED_GENERAL_DEFINE,

    // CPU LSU IQ 中的请求接口
    input  logic        lsu_req_valid_i,
    output logic        lsu_req_ready_o,
    input  iq_lsu_req_t lsu_req_i,

    output logic         lsu_resp_valid_o,
    input  logic         lsu_resp_ready_i,
    output iq_lsu_resp_t lsu_resp_o,

    // CPU COMMIT 中的交互端口
    input commit_lsu_req_t  commit_lsu_req_i,
    input commit_lsu_resp_t commit_lsu_resp_o,

    // 到总线侧的请求接口
    output lsu_bus_req_t  bus_req_o,
    input  lsu_bus_resp_t bus_resp_i,

    // 用于 SNOOP 的总线更新接口
    input  dsram_snoop_t  snoop_i,

    // 用于地址翻译更新接口
    input  tlb_update_req_t tlb_update_i,

    // 考虑 icache 每周期需要读取多个数据？
    // 需要大位宽 sram 端口 -> 串行访问？ 以一个周期为额外开销，Perfect！
    output logic[11:2]      sram_raddr, // READ ONLY PORT
    input  logic[3:0][31:0] sram_rdata, // 32b sram port
    input  d_tag_t [3:0]    sram_rtag
);
    // 同时开始地址翻译请求与 SRAM 请求。
endmodule
