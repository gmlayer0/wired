`include "wired0_defines"

// Fuction module for Wired project
// Commit module, fetch all static and dynamic information about instruction execution.
// Futhermore, this module will change CSR / ARF / NPC status to commit instruction.
// This module can commit(retire) at most two inst / cycle.
// Because our CDB is 2-width, this is already enough.
module wired_commit #(
    parameter int INST_PAYLOAD_SIZE = 128
)(
    `_WIRED_GENERAL_DEFINE,

    // Part 1: ROB 读端口
    output rob_rid_t         [1:0] c_rrrid_o,
    input  logic             [1:0] c_rob_valid_i,
    input  rob_entry_t       [1:0] c_rob_entry_i,
    output logic             [1:0] c_retire_o,

    // Part 2: LSU 提交请求端口
    output commit_lsu_req_t        c_lsu_req_o,
    input  commit_lsu_resp_t       c_lsu_resp_i,

    // Part 3: ARF / Rename 更新端口
    output      logic        [1:0] l_retire_o, // FOR RENAME
    output      logic        [1:0] l_commit_o, // FOR ARF
    output     word_t        [1:0] l_data_o,
    output arch_rid_t        [1:0] l_warid_o,
    output  rob_rid_t        [1:0] l_wrrid_o,
    output      logic        [1:0] l_tier_id_o,

    // Part 4: 前端更新端口（跳转/flush/异常中断处理）
    output bpu_correct_t           f_upd_o,

    // Part 5: flush 端口，刷新流水线用
    output      logic              l_flush_o,
    input       logic              rename_empty_i, // RENAME 级清空，说明 ROB 均已退休，ROB 空，可以继续。

    // Part 6: CSR 及地址翻译相关控制接口
    // 注意，提交模块分为三级，分别是 Fetch ROB(F) | Handle(H) | Commit(C)
);

    // 第一级流水，F，需要判定是否可双提交。
    // 对于可能需要状态机处理或者修改 CSR 、产生跳转、刷新流水线效果的指令，仅允许在 SLOT0 提交。
    // 其它指令则无所谓
    rob_rid_t f_rob_ptr0_q;
    rob_rid_t f_rob_ptr1_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            f_rob_ptr0_q <= `_WIRED_PARAM_ROB_LENd'0;
            f_rob_ptr1_q <= `_WIRED_PARAM_ROB_LENd'1;
        end else if(f_skid_ready_q) begin
            f_rob_ptr0_q <= f_rob_ptr0_q + f_valid[0] + f_valid[1];
            f_rob_ptr1_q <= f_rob_ptr1_q + f_valid[0] + f_valid[1];
        end
    end
    reg f_skid_ready_q;
    rob_entry_t [1:0] f_skid_entry_q;
    wire slot1_bank_conflict = c_rob_entry_i[1].wreg[0] == c_rob_entry_i[0].wreg[0];
    wire slot1_cannot_fire = c_rob_entry_i[1].decode_info.slot0 || slot1_bank_conflict1;
    wire [1:0] f_valid = c_rob_valid_i & {~slot1_cannot_fire, 1'd1};
    assign c_retire_o = f_valid & {f_skid_ready_q, f_skid_ready_q};

    always_ff @(posedge clk) begin
        if(f_skid_ready_q) begin
            f_skid_entry_q <= c_rob_entry_i;
        end
    end

    // 一二级流水之间插入的 Skid Buffer
    wire h_ready;    // H 级准备好接受来自 F 级别的数据
    reg [1:0] f_skid_valid_q;
    rob_entry_t [1:0] h_entry;
    assign h_entry = f_skid_ready_q ? c_rob_entry_i : f_skid_entry_q;
    wire [1:0] h_valid = f_skid_ready_q ? f_valid : f_skid_valid_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            f_skid_ready_q <= '1;
            f_skid_valid_q <= '0;
        end else if(f_skid_ready_q) begin
            if(!h_ready && (|f_valid)) begin
                f_skid_ready_q <= '0;
                f_skid_valid_q <= f_valid;
            end
        end else begin
            if(h_ready) begin
                f_skid_ready_q <= '1;
                f_skid_valid_q <= '0;
            end
        end
    end

    // 第二级流水（实质是一个状态机）
    rob_entry_t [1:0] h_entry_q;
    logic [1:0]  h_valid_inst_q;
    always_ff @(posedge clk) begin
        if(h_ready) begin
            h_entry_q <= h_entry;
        end
    end
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            h_valid_inst_q <= '0;
        end else begin
            if(h_ready) begin
                h_valid_inst_q <= h_valid;
            end
        end
    end

    // 主提交状态机
    // 需要考虑：
    // 1. Uncached 请求（向 LSU 发出请求，等待 Uncached 读数据 / 写完成）
    // （注意， Uncached Load 及 Uncached Store 均需要立即发出请求，且均需要刷新流水线）
    // 2. Store 请求，等待 Cache 重填后再执行（暂停等待）
    // 3. Store Conditional 请求，不再等待 Cache 重填，如果不可执行，则直接刷新流水线
    // 4. CSRWR/CSRRD 指令，直接刷新管线
    // （注意， Store Conditional 请求需要检查 store buffer 中的情况）
    // 经过之前的约束，所有可能造成暂停的指令（Uncache load/store | Missed Store）均一定在槽一
    // 可能产生流水线刷新的指令也是同样的情况
    csr_t csr, csr_init;
    logic[63:0] timer_64_q;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            timer_64_q <= '0;
        end else begin
            timer_64_q <= timer_64_q + 64'b1;
        end
    end
    always_comb begin
        csr_init = '0;
        csr_init.crmd[`_CRMD_DA] = 1'd1; // 初始化要求非0的 CSR 寄存器值
        csr_init.asid[31:10] = 22'h280;
    end
    csr_t csr_q;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            csr_q <= csr_init; // 初始化 CSR
        end else if(h_ready) begin
            csr_q <= csr;
        end
    end

    // 注意从此开始不再需要握手，因为之后的流水线一路通畅。
    logic            l_flush;  // 流水线需要冲刷
    logic      [1:0] l_retire; // 标识指令的信息需要进入 Rename，注意，所有进入 ROB 的指令无论实际是否执行一定需要 retire。
    logic      [1:0] l_commit; // 标识指令的结果需要进入 CSR / 写入 ARF
    word_t     [1:0] l_data;
    arch_rid_t [1:0] l_warid;
    rob_rid_t  [1:0] l_wrrid;
    logic      [1:0] l_tier_id;
    bpu_correct_t    f_upd;
    always_ff @(posedge clk) begin
        l_flush_o <= l_flush;
        l_retire_o <= l_retire;
        l_commit_o <= l_commit;
        l_data_o <= l_data;
        l_warid_o <= l_warid;
        l_wrrid_o <= l_wrrid;
        l_tier_id_o <= l_tier_id;
        f_upd_o <= f_upd;
    end

    // 主状态机定义
    `define __FSM_BIT_LEN 5
    typedef logic[`__FSM_BIT_LEN-1:0] commit_fsm_t;
    parameter commit_fsm_t S_NORMAL      = `__FSM_BIT_LEN'd0;
    parameter commit_fsm_t S_WAIT_ULOAD  = `__FSM_BIT_LEN'd1; // 这个要刷流水线
    parameter commit_fsm_t S_WAIT_USTORE = `__FSM_BIT_LEN'd2; // 这个要刷流水线，还要修改写入 ARF 的数据
    parameter commit_fsm_t S_WAIT_MSTORE = `__FSM_BIT_LEN'd3; // 这个不用刷流水线
    parameter commit_fsm_t S_WAIT_FLUSH  = `__FSM_BIT_LEN'd4; // 这个是用来刷流水线的
    commit_fsm_t fsm_q;
    commit_fsm_t fsm;
    always_ff @(posedge clk) begin
        if(~rst_n) begin
            fsm_q <= S_NORMAL;
        end else begin
            fsm_q <= fsm;
        end
    end

    // 主状态机组合逻辑
    always_comb begin
        csr = csr_q;
        fsm = fsm_q;
        l_flush = '0;  // 流水线需要冲刷
        l_retire = '0; // 标识指令的信息需要进入 Rename，注意，所有进入 ROB 的指令无论实际是否执行一定需要 retire。
        l_commit = '0; // 标识指令的结果需要进入 CSR / 写入 ARF
        l_data = 'x;
        l_warid = 'x;
        l_wrrid = 'x;
        l_tier_id = 'x;
        f_upd = 'x;
        f_upd.true_taken = '0;
        f_upd.miss = '0;
        f_upd.pc = '0;
    end

endmodule
