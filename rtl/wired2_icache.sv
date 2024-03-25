// icache cpu side
`include "wired0_defines"

module wired_icache #(
    parameter integer PACKED_SIZE = 32
)(
    `_WIRED_GENERAL_DEFINE,

    // CPU FETCH 请求接口
    input  logic                   f_valid_i,
    output logic                   f_ready_o,
    input  logic                [31:0]  pc_i,
    input  logic [PACKED_SIZE - 1 : 0] pkg_i,

    // CACHE OP 请求端口
    input  logic                   c_valid_i,
    output logic                   c_ready_o,
    input  logic            [31:0]  c_addr_i,
    input 

    // 到总线侧的请求接口
    output lsu_bus_req_t  bus_req_o,
    input  lsu_bus_resp_t bus_resp_i,

    // 用于 SNOOP 的总线更新接口
    input  dsram_snoop_t  snoop_i,

    // 用于地址翻译更新接口
    input  csr_t            csr_i,
    input  tlb_update_req_t tlb_update_i,

    // SRAM 读端口
    output logic [11:0]       p_addr_o,
    input  logic [3:0][31:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,
    input  logic [1:0]        p_sll_i,

    // 无效化端口
    input  logic              flush_i

);

endmodule
