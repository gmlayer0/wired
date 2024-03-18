// dcache cpu side
`include "wired0_defines"

module wired_tl_adapter #(
    parameter int unsigned SourceWidth = 1,
    parameter int unsigned SinkWidth   = 1,
    parameter bit [SourceWidth-1:0] SourceBase  = 0,
)(
    `_WIRED_GENERAL_DEFINE,
    // 到 CPU 侧的请求接口
    input  lsu_bus_req_t  bus_req_i,
    output lsu_bus_resp_t bus_resp_o,

    // 用于 SNOOP 的总线更新接口
    output dsram_snoop_t  snoop_i,

    // DSRAM 端口
    output  logic [1:0]  m_way_o,
    output  logic [11:0] m_addr_o,       // 状态机侧访问地址，读写某 way 的整行
    output  logic [3:0][3:0] m_wstrb_o,  // 写掩码，全 0 为读
    output  logic [3:0][31:0] m_wdata_o,
    input   logic [3:0][31:0] m_rdata_i,   // 整行，已内部对齐（注：组合逻辑对齐）

    // TSRAM 端口
    output  logic [11:4] t_addr_o,
    output  logic  [3:0] t_we_o,
    output  cache_tag_t  t_wtag_o,
    input   cache_tag_t  t_rtag_i,

    `TL_DECLARE_HOST_PORT(DataWidth, PhysAddrLen, SourceWidth, SinkWidth, tl) // tl_a_o
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
    logic        crq_unc_wreq; // crq drive, write/read selection.
    logic  [3:0] crq_unc_strb; // crq drive
    logic  [1:0] crq_unc_size; // crq drive
    logic [31:0] crq_unc_addr; // crq drive
    logic [31:0] crq_unc_data; // unc drive

    /* --- --- --- --- --- --- --- --- FSM Begin --- --- --- --- --- --- --- --- --- */

    // 有意思的是，只有 A 需要仲裁，C、E 均为独享。
    typedef `TL_A_STRUCT(128, 32, SourceWidth, SinkWidth) tl_a_t;
    typedef `TL_B_STRUCT(128, 32, SourceWidth, SinkWidth) tl_b_t;
    typedef `TL_C_STRUCT(128, 32, SourceWidth, SinkWidth) tl_c_t;
    typedef `TL_D_STRUCT(128, 32, SourceWidth, SinkWidth) tl_d_t;
    typedef `TL_E_STRUCT(128, 32, SourceWidth, SinkWidth) tl_e_t;

    // Probe       状态机 - prb - B - read SRAM-TAG
    /* - tl - */
    logic prb_b_valid, prb_b_ready;
    tl_b_t prb_b;
    /* - sram - */
    logic prb_tag_valid, prb_tag_ready;
    logic [11:4] prb_tag_set;
    cache_tag_t [3:0] prb_tag;

    // CPU_Request 状态机 - crq

    // Invalid     状态机 - inv - C、D - write SRAM-TAG, read SRAM-DATA
    /* - tl - */
    logic inv_c_valid, inv_c_ready;
    tl_c_t inv_c;
    logic inv_d_valid, inv_d_ready;
    tl_d_t inv_d;
    /* - sram - */
    logic inv_tag_valid, inv_tag_ready;
    logic [11:4] inv_tag_set;
    logic  [3:0] inv_tag_we;
    cache_tag_t  inv_tag;

    // Acquire     状态机 - acq - A、D、E - write SRAM-TAG, write SRAM-DATA
    /* - tl - */
    logic acq_a_valid, acq_a_ready;
    tl_a_t acq_a;
    logic acq_d_valid, acq_d_ready;
    tl_d_t acq_d;
    logic acq_e_valid, acq_e_ready;
    tl_e_t acq_e;
    /* - sram - */
    logic acq_tag_valid, acq_tag_ready;
    logic [11:4] acq_tag_set;
    logic  [3:0] acq_tag_we;
    cache_tag_t  acq_tag;

    // Uncached    状态机 - unc - A、D
    logic unc_a_valid, unc_a_ready;
    tl_a_t unc_a;
    logic unc_d_valid, unc_d_ready;
    tl_d_t unc_d;

    /* --- --- --- --- --- --- --- --- ARB Begin --- --- --- --- --- --- --- --- --- */

    // SRAM 仲裁器，固定优先级

    // TILELINK 仲裁器，固定优先级

    // A - acq unc
    tl_a_t [1:0] tl_a_mult;
    logic  [1:0] tl_a_mult_valid;
    logic  [1:0] tl_a_mult_ready;
    assign tl_a_mult[0] = acq_a;
    assign tl_a_mult_valid[0] = acq_a_valid;
    assign acq_a_ready = tl_a_mult_ready[0];
    assign tl_a_mult[1] = unc_a;
    assign tl_a_mult_valid[1] = unc_a_valid;
    assign unc_a_ready = tl_a_mult_ready[1];

    // 接出，固定优先级
    assign tl_a_valid_o = |tl_a_mult_valid;
    assign tl_a_o = tl_a_mult_valid[0] ? tl_a_mult[0] : tl_a_mult[1];
    assign tl_a_mult_ready[0] = tl_a_ready_i;
    assign tl_a_mult_ready[1] = tl_a_mult_valid[0] ? '0 : tl_a_ready_i;

    // D - inv acq unc
    assign inv_d = tl_d_i;
    assign acq_d = tl_d_i;
    assign unc_d = tl_d_i;
    assign inv_d_valid = tl_d_valid_i;
    assign acq_d_valid = tl_d_valid_i;
    assign unc_d_valid = tl_d_valid_i;
    assign tl_d_ready_o = inv_d_ready | acq_d_ready | unc_d_ready;

    // B - prb
    assign prb_b = tl_b_i;
    assign prb_b_valid = tl_b_valid_i;
    assign tl_b_ready_o = prb_b_ready;

    // C - inv
    assign tl_c_o = inv_c;
    assign tl_c_valid_o = inv_c_valid;
    assign inv_c_ready = tl_c_ready_i;

    // E - acq
    assign tl_e_o = acq_e;
    assign tl_e_valid_o = acq_e_valid;
    assign acq_e_ready = tl_e_ready_i;

endmodule
