// icache cpu side
`include "wired0_defines"

module wired_icache #(
    parameter integer PACKED_SIZE = 32
)(
    `_WIRED_GENERAL_DEFINE,

    // CPU FETCH 请求接口
    input  logic                   f_valid_i,
    output logic                   f_ready_o,
    input  logic            [1:0]   f_mask_i,
    input  logic            [31:0]    f_pc_i,
    input  logic [PACKED_SIZE-1 : 0] f_pkg_i,

    // CACHE OP 请求端口
    input  logic                   c_valid_i,
    output logic                   c_ready_o,
    input  logic            [31:0]  c_addr_i,
    input  inv_parm_e               c_parm_i,

    // CPU FETCH 请求返回接口
    output logic                   f_valid_o,
    input  logic                   f_ready_i,
    output logic            [1:0]   f_mask_o,
    output logic            [31:0]    f_pc_o,
    output logic       [1:0][31:0]  f_inst_o,
    output logic [PACKED_SIZE-1 : 0] f_pkg_o,

    // 到总线侧的请求接口
    output lsu_bus_req_t  bus_req_o, // TODO
    input  lsu_bus_resp_t bus_resp_i,

    // 用于 SNOOP 的总线更新接口
    input  dsram_snoop_t  snoop_i,

    // 用于地址翻译更新接口
    input  csr_t            csr_i,
    input  tlb_update_req_t tlb_update_i,

    // SRAM 读端口
    output logic [11:0]       p_addr_o, // TODO
    input  logic [3:0][64:0]  p_rdata_i,
    input  cache_tag_t [3:0]  p_tag_i,
    input  logic [1:0]        p_sll_i,

    // 无效化端口
    input  logic              flush_i
);
    logic f1_skid_busy_q; // F1 级别进入 skid 状态的信号
    assign f_ready_o = (!f1_skid_busy_q) && (!c_valid_i);
    assign c_raedy_o = !f1_skid_busy_q;
    // F2 段到 f1 握手
    logic f1_f2_valid, f1_f2_ready; // 握手信号，纯组合逻辑驱动
    typedef struct packed {
        logic [31:0] addr;
        inv_parm_e   parm;
        logic [1:0]  mask;
        logic     uncache;

        logic [PACKED_SIZE - 1 : 0] pkg; // 时刻携带传入信息

        cache_tag_t [3:0]  tag;
        logic [3:0][31:0] data;
        logic inv; logic ppi; logic ale; logic tlbr; // TLB EXCP
    } f1_pack_t;
    f1_pack_t f1_raw, f1_sel, f1; // 组合逻辑
    f1_pack_t f1_skid_q; // 寄存器堆
    logic [3:0] f1_hit;  // 纯组合逻辑，基于 f1 信号
    always_ff @(posedge clk) begin
        if(!f1_skid_busy_q) begin
            f1_skid_q <= f1; // 每拍都打
        end
    end
    assign f1_sel = f1_skid_busy_q ? f1_skid_q : f1_raw;
    
    // SNOP 逻辑开始
    always_comb begin
        f1 = f1_sel;
        // TODO: SNOOP SRAM SIGNALS
    end

    // F1_RAW 逻辑开始
    always_comb begin
        f1_raw = '0;
        // TODO: RAW GENERATION LOGICS
    end

    // HIT 逻辑开始
    for(genvar w = 0 ; w < 4 ; w += 1) begin
        assign f1_hit[w] = '0; // 生成四路 hit信号。
        // TODO        
    end
    /* F2 段开始 */
    logic f2_valid_q; // F2 有效寄存器，状态机使用
    typedef struct packed {
        logic [31:0] addr;
        inv_parm_e   parm;
        logic [1:0]  mask;
        logic     uncache;

        logic [PACKED_SIZE - 1 : 0] pkg; // 时刻携带传入信息

        logic [3:0]   hit; // 是否命中信息
        logic [3:0][31:0] data; // 需要实时更新
        logic inv; logic ppi; logic ale; logic tlbr; // TLB EXCP
    } f2_pack_t;
    typedef enum logic[3:0] {

    } fsm_t;
    fsm_t fsm;
    fsm_t fsm_q;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            fsm_q <= '0;
        end else begin
            fsm_q <= fsm;
        end
    end
    always_comb begin
        fsm = fsm_q;
        // 之后是主要状态机
    end


endmodule
