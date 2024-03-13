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
    input  csr_t            csr_i,
    input  tlb_update_req_t tlb_update_i,

    // SRAM 读端口
    output logic [11:0]       p_addr_o,
    input  logic [3:0][31:0]  p_rdata_i,
    input  cache_tage_t [3:0] p_tag_i,
    input  logic [1:0]        p_sll_i,

    // Probe 端口
    input sram_snoop_t        m_snoop_i

);
    // M1 信号定义
    logic iq_m1_ready; // iq -> m1 的通路就绪
    tlb_s_resp_t m1_tlb_resp_raw;
    // 同时开始地址翻译请求与 SRAM 请求。
    // 例化地址翻译模块
    wired_addr_trans # (
        .FETCH_ADDR('0)
    )
    wired_addr_trans_inst (
        `_WIRED_GENERAL_CONN,
        .clken_i(iq_m1_ready),
        .vaddr_i(lsu_req_i.vaddr),
        .csr_i(csr_i),
        .tlb_update_req_i(tlb_update_i),
        .trans_result_o(m1_tlb_resp_raw)
    );

    // M1 输出部分
    // 握手生成
    assign lsu_req_ready_o = iq_m1_ready;

    // 注： M1 天然带有 skid buf 属性，当 M2 暂停后的第一个周期，M1 依然可以接受一条请求。
    // M1 此时的请求将被压入 M1 的 skid buf 中记录，并实时检查 snoop 进行更新。
    // 这里的 snoop 主要是为了避免重复 refill

    // 正常的路线不需要检查 snoop，仅检查 store buffer 即可。
    // 
endmodule
