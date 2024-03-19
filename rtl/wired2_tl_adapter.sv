// dcache bus side
`include "wired0_defines"

// include tilelink header
`include "tl_util.svh"

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

    // CPU_Request 状态机 - crq - - write SRAM-DATA
    /* - data sram - */
    logic             crq_data_valid, crq_data_ready;
    logic  [1:0]      crq_data_way;
    logic [11:0]      crq_data_addr;
    logic [3:0][3:0]  crq_data_wstrb;
    logic [3:0][31:0] crq_data_wdata;

    // Invalid     状态机 - inv - C、D - write SRAM-TAG, read SRAM-DATA
    /* - tl - */
    logic inv_c_valid, inv_c_ready;
    tl_c_t inv_c;
    logic inv_d_valid, inv_d_ready;
    tl_d_t inv_d;
    /* - tag sram - */
    logic inv_tag_valid, inv_tag_ready;
    logic [11:4] inv_tag_set;
    logic  [3:0] inv_tag_we;
    cache_tag_t  inv_tag;
    /* - data sram - */
    logic        inv_data_valid, inv_data_ready;
    logic  [1:0] inv_data_way;
    logic [11:0] inv_data_addr;
    logic [3:0][31:0] inv_data_rdata;

    // Acquire     状态机 - acq - A、D、E - write SRAM-TAG, write SRAM-DATA
    /* - tl - */
    logic acq_a_valid, acq_a_ready;
    tl_a_t acq_a;
    logic acq_d_valid, acq_d_ready;
    tl_d_t acq_d;
    logic acq_e_valid, acq_e_ready;
    tl_e_t acq_e;
    /* - tag sram - */
    logic acq_tag_valid, acq_tag_ready;
    logic [11:4] acq_tag_set;
    logic  [3:0] acq_tag_we;
    cache_tag_t  acq_tag;
    /* - data sram - */
    logic             acq_data_valid, acq_data_ready;
    logic  [1:0]      acq_data_way;
    logic [11:0]      acq_data_addr;
    logic [3:0][3:0]  acq_data_wstrb;
    logic [3:0][31:0] acq_data_wdata;

    // Uncached    状态机 - unc - A、D
    logic unc_a_valid, unc_a_ready;
    tl_a_t unc_a;
    logic unc_d_valid, unc_d_ready;
    tl_d_t unc_d;

    /* --- --- --- --- --- --- --- --- ARB Begin --- --- --- --- --- --- --- --- --- */

    // SRAM 仲裁器，固定优先级
    // - tag 仲裁器，两写一读 - prb / inv acq -
    logic [2:0] sram_tag_valid_mult, sram_tag_ready_mult;
    logic [2:0][11:4] sram_tag_addr_mult;
    logic [2:0][3:0]  sram_tag_we_mult;
    cache_tag_t [2:0] sram_tag_w_mult;
    assign sram_tag_valid_mult[0] = prb_tag_valid;
    assign sram_tag_valid_mult[1] = inv_tag_valid;
    assign sram_tag_valid_mult[2] = acq_tag_valid;
    assign sram_tag_addr_mult[0] = prb_tag_set;
    assign sram_tag_addr_mult[1] = inv_tag_set;
    assign sram_tag_addr_mult[2] = acq_tag_set;
    assign prb_tag_ready = sram_tag_ready_mult[0];
    assign inv_tag_ready = sram_tag_ready_mult[1];
    assign acq_tag_ready = sram_tag_ready_mult[2];
    assign sram_tag_we_mult[0] = '0;
    assign sram_tag_we_mult[1] = inv_tag_we;
    assign sram_tag_we_mult[2] = acq_tag_we;
    assign sram_tag_w_mult[0] = '0;
    assign sram_tag_w_mult[1] = inv_tag;
    assign sram_tag_w_mult[2] = acq_tag;
    assign prb_tag = t_rtag_i;
    assign sram_tag_ready_mult[0] = '1;
    assign sram_tag_ready_mult[1] = ~sram_tag_valid_mult[0];
    assign sram_tag_ready_mult[2] = ~sram_tag_valid_mult[0] & ~sram_tag_valid_mult[1];
    assign t_addr_o = sram_tag_valid_mult[0] ? sram_tag_addr_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_addr_mult[1] :
                                               sram_tag_addr_mult[2];
    assign t_we_o   = sram_tag_valid_mult[0] ? sram_tag_we_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_we_mult[1] :
                                               sram_tag_we_mult[2];
    assign t_wtag_o = sram_tag_valid_mult[0] ? sram_tag_w_mult[0] : 
                      sram_tag_valid_mult[1] ? sram_tag_w_mult[1] :
                                               sram_tag_w_mult[2];
    // - data 仲裁器，两写一读 - inv / crq acq -
    logic [2:0] sram_data_valid_mult, sram_data_ready_mult;
    logic [2:0][1:0]  sram_data_way_mult;
    logic [2:0][11:0] sram_data_addr_mult;
    logic [2:0][3:0][3:0]  sram_data_strb_mult;
    logic [2:0][3:0][31:0] sram_data_w_mult;
    assign sram_data_valid_mult[0] = crq_data_valid;
    assign sram_data_valid_mult[1] = inv_data_valid;
    assign sram_data_valid_mult[2] = acq_data_valid;
    assign sram_data_way_mult[0] = crq_data_way;
    assign sram_data_way_mult[1] = inv_data_way;
    assign sram_data_way_mult[2] = acq_data_way;
    assign sram_data_addr_mult[0] = crq_data_addr;
    assign sram_data_addr_mult[1] = inv_data_addr;
    assign sram_data_addr_mult[2] = acq_data_addr;
    assign crq_data_ready = sram_data_ready_mult[0];
    assign inv_data_ready = sram_data_ready_mult[1];
    assign acq_data_ready = sram_data_ready_mult[2];
    assign sram_data_strb_mult[0] = crq_data_wstrb;
    assign sram_data_strb_mult[1] = '0;
    assign sram_data_strb_mult[2] = acq_data_wstrb;
    assign sram_data_w_mult[0] = crq_data_wdata;
    assign sram_data_w_mult[1] = '0;
    assign sram_data_w_mult[2] = acq_data_wdata;
    assign inv_data_rdata = m_rdata_i;
    assign sram_data_ready_mult[0] = '1;
    assign sram_data_ready_mult[1] = ~sram_data_valid_mult[0];
    assign sram_data_ready_mult[2] = ~sram_data_valid_mult[0] & ~sram_data_valid_mult[1];
    assign m_way_o   = sram_data_valid_mult[0] ? sram_data_way_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_way_mult[1] :
                                                sram_data_way_mult[2];
    assign m_addr_o  = sram_data_valid_mult[0] ? sram_data_addr_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_addr_mult[1] :
                                                sram_data_addr_mult[2];
    assign m_strb_o  = sram_data_valid_mult[0] ? sram_data_strb_mult[0] :
                      sram_data_valid_mult[1] ? sram_data_strb_mult[1] :
                                                sram_data_strb_mult[2];
    assign m_wdata_o = sram_data_valid_mult[0] ? sram_data_w_mult[0] :
                       /*sram_data_valid_mult[1] ? sram_data_w_mult[1] : 实际上不太需要*/
                                                sram_data_w_mult[2];

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
