// dcache cpu side
`include "wired0_defines"

module wired_lsu(
    `_WIRED_GENERAL_DEFINE,
    // 到 CPU 侧的请求接口
    input  lsu_bus_req_t  bus_req_i,
    output lsu_bus_resp_t bus_resp_o,

    // 用于 SNOOP 的总线更新接口
    output dsram_snoop_t  snoop_i,

    // SRAM 端口
    output  logic [1:0]  m_way_o,
    output  logic [11:0] m_addr_o,       // 状态机侧访问地址，读写某 way 的整行
    output  logic [3:0][3:0] m_wstrb_o,  // 写掩码，全 0 为读
    output  logic [3:0][31:0] m_wdata_o,
    input logic [3:0][31:0] m_rdata_i    // 整行，已内部对齐（注：组合逻辑对齐）
);

    /* --- --- --- --- --- --- ---  I-FSM-CALL Begin  --- --- --- --- --- --- --- --- */
    // Inter-FSM Called signals
    // - prb begin
    // --- prb call inv
    logic prb_inv_cal; // prb drive
    logic prb_inv_ret; // inv drive
    // payload
    logic [1:0] prb_inv_way; // prb drive
    logic [7:0] prb_inv_set; // prb drive
    // - crq begin
    // --- crq call inv
    logic crq_inv_cal; // crq drive
    logic crq_inv_ret; // inv drive
    // payload
    logic [1:0]  crq_inv_way; // crq drive
    logic [11:4] crq_inv_set; // crq drive
    // --- crq call acq
    logic crq_acq_cal; // crq drive
    logic crq_acq_ret; // acq drive
    // payload
    logic             crq_acq_wp;   // crq drive, wether to get write permission
    logic [1:0]       crq_acq_way;  // crq drive
    logic [31:4]      crq_acq_addr; // crq drive
    logic [3:0][31:0] crq_acq_data; // acq drive
    // --- crq call unc
    logic crq_unc_cal; // crq drive
    logic crq_unc_ret; // unc drive
    // payload
    logic  [3:0] crq_unc_strb; // crq drive, wether to get write permission
    logic  [1:0] crq_unc_size; // crq drive
    logic [31:0] crq_unc_addr; // crq drive
    logic [31:0] crq_unc_data; // unc drive

    /* --- --- --- --- --- --- --- --- FSM Begin --- --- --- --- --- --- --- --- --- */

    // Probe       状态机 - prb - B 
    logic prb_b_valid, prb_b_ready;

    // CPU_Request 状态机 - crq

    // Invalid     状态机 - inv - C、D
    logic inv_c_valid, inv_c_ready;
    logic inv_d_valid, inv_d_ready;

    // Acquire     状态机 - acq - A、D、E
    logic acq_a_valid, acq_a_ready;
    logic acq_d_valid, acq_d_ready;
    logic acq_e_valid, acq_e_ready;

    // Uncached    状态机 - unc - A、D
    logic unc_a_valid, unc_a_ready;
    logic unc_d_valid, unc_d_ready;

    /* --- --- --- --- --- --- --- --- ARB Begin --- --- --- --- --- --- --- --- --- */

    // SRAM 仲裁器，固定优先级

    // TILELINK 仲裁器，固定优先级

    // A - acq unc

    // B - prb

    // C - inv

    // D - inv acq unc

    // E - acq

endmodule
