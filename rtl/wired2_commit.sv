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
    rob_rid_t [1:0] f_skid_wrrid_q;
    wire slot1_bank_conflict = c_rob_entry_i[1].wreg[0] == c_rob_entry_i[0].wreg[0];
    wire slot1_cannot_fire = c_rob_entry_i[1].decode_info.slot0 || slot1_bank_conflict1;
    wire [1:0] f_valid = c_rob_valid_i & {~slot1_cannot_fire, 1'd1};
    assign c_retire_o = f_valid & {f_skid_ready_q, f_skid_ready_q};

    always_ff @(posedge clk) begin
        if(f_skid_ready_q) begin
            f_skid_entry_q <= c_rob_entry_i;
            f_skid_wrrid_q <= {f_rob_ptr1_q, f_rob_ptr0_q};
        end
    end

    // 一二级流水之间插入的 Skid Buffer
    logic h_ready;    // H 级准备好接受来自 F 级别的数据
    reg [1:0] f_skid_valid_q;
    rob_entry_t [1:0] h_entry;
    rob_rid_t [1:0] h_wrrid;
    assign h_entry = f_skid_ready_q ? c_rob_entry_i : f_skid_entry_q;
    assign h_wrrid = f_skid_ready_q ? {f_rob_ptr1_q, f_rob_ptr0_q} : f_skid_wrrid_q;
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
    rob_rid_t [1:0] h_wrrid_q;
    logic [1:0]  h_valid_inst_q;
    rob_rid_t [1:0] h_wrrid_q;
    always_ff @(posedge clk) begin
        if(h_ready) begin
            h_entry_q <= h_entry;
            h_wrrid_q <= h_wrrid;
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
    // 4. Branch 类型指令，预测错误时跳转并更新 BPU
    // 5. CSRWR/CSRRD 指令，直接刷新管线
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
    logic      [1:0] l_commit; // 标识指令的结果需要写入 ARF
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
    parameter commit_fsm_t S_WAIT_ULOAD  = `__FSM_BIT_LEN'd1; // 这个要刷流水线，还要修改写入 ARF 的数据
    parameter commit_fsm_t S_WAIT_USTORE = `__FSM_BIT_LEN'd2; // 这个不用刷流水线
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
    excp_t [1:0] excp;
    for(genvar i = 0 ; i < 2 ; i += 1) begin
        assign excp[i] = gather_excp(h_entry_q[i].static_excp, h_entry_q[i].lsu_excp);
    end
    always_comb begin
        c_lsu_req_o = '0;
        csr = csr_q;
        fsm = fsm_q;
        h_ready = '1;
        l_flush = '0;  // 流水线需要冲刷
        l_retire = '0; // 标识指令的信息需要进入 Rename，注意，所有进入 ROB 的指令无论实际是否执行一定需要 retire。
        l_commit = '0; // 标识指令的结果需要进入 CSR / 写入 ARF
        for(integer i = 0 ; i < 2 ; i+=1) begin
            l_data[i] = h_entry_q[i].wdata;
            l_warid[i] = h_entry_q[i].wreg;
            l_wrrid[i] = h_wrrid_q[i];
            l_tier_id[i] = h_entry_q[i].wtier;
        end
        f_upd = '0;
        case (fsm_q)
            S_NORMAL: begin
                l_flush = '0;
                l_retire = h_valid_inst_q;
                l_commit = h_valid_inst_q;
                // 对 inst 的中断异常情况做 judge
                if(h_valid_inst_q[0] && |excp[0] || h_valid_inst_q[1] && |excp[1]) begin
                    for(integer i = 1 ; i >= 0 ; i --) begin
                        if(h_valid_inst_q[i] && |excp[i]) begin
                            // 第 i 条指令有问题
                            l_commit[1] = '0;
                            l_commit[i] = '0;
                            fsm = S_WAIT_FLUSH;
                            if(i != 1 || (!|excp[0])) begin
                                // 对于 0 一定可以修改，对于1，一定在 0 无异常时可以修改
                                // 先更新 ecode
                                case(1'b1)
                                    excp[i].fetch_int:      // None Masked Interruption founded, if founded, this instruction is forced to issue in ALU slot
                                    excp[i].adef:
                                    excp[i].itlbr:
                                    excp[i].pif:
                                    excp[i].ippi:
                                    excp[i].ine:
                                    excp[i].ipe:
                                    excp[i].sys:
                                    excp[i].brk:
                                    excp[i].ale:
                                    excp[i].tlbr:
                                    excp[i].pis:
                                    excp[i].pil:
                                    excp[i].ppi:
                                    excp[i].pme:
                                endcase
                            end
                        end
                    end
                end else begin
                    // 不存在异常的部分
                    // 0. 恢复 DBAR
                    if(h_entry_q[0].decode_info.dbarrier) begin
                        c_lsu_req_o.dbarrier_unlock = '1;
                    end
                    // 访存部分处理
                    if(h_entry_q[0].lsu_inst) begin
                        if(h_entry_q[0].uncached) begin
                            // 1. Uncached 请求（向 LSU 发出请求，等待 Uncached 读数据 / 写完成）
                            // （注意， Uncached Load 及 Uncached Store 均需要立即发出请求，且 Uncached Load 需要刷新流水线）
                            h_ready = '0;
                            if(h_entry_q[0].decode_info.mem_write) begin
                                fsm = S_WAIT_USTORE;
                            end else begin
                                fsm = S_WAIT_ULOAD;
                            end
                        end else begin
                            if(h_entry_q[0].decode_info.mem_write) begin
                                // 3. Store Conditional 请求，不再等待 Cache 重填，如果不可执行，则直接刷新流水线
                                if(h_entry_q[0].decode_info.llsc_inst) begin
                                    if((!c_lsu_resp_i.storebuf_hit) || (!csr_q.llbit)) begin // 其它 Cache Coherent Master probe 了这行，不再原子
                                        l_commit = 2'b01;
                                        l_data[0] = 32'd0;
                                        fsm = S_WAIT_FLUSH; // 对于失败的 SC ，需要 refresh 流水线
                                    end else begin
                                        // 成功的 SC，弹栈
                                        c_lsu_req_o.storebuf_commit = '1;
                                    end
                                    csr.llbit = '0; // 下周期清理 llbit
                                end else begin
                                    // 2. 非 SC 的 Store 请求 MISS ，等待 Cache 重填后再执行（暂停等待）
                                    if(!c_lsu_resp_i.storebuf_hit) begin
                                        h_ready = '0;
                                        fsm = S_WAIT_USTORE;
                                    end else begin
                                        c_lsu_req_o.storebuf_commit = '1;
                                    end
                                end
                            end
                        end
                    end
                end

                // 跳转指令处理
                
                // CSR 指令处理，注意刷新管线
            end
            S_WAIT_ULOAD: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.uncached_load_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = 2'b01;
                    l_data[0] = c_lsu_resp_i.uncached_load_resp;
                    fsm = S_WAIT_FLUSH; // 对于 Uncached load ，需要 refresh 流水线
                end
            end
            S_WAIT_USTORE: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.uncached_store_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = h_valid_inst_q;
                    c_lsu_req_o.storebuf_commit = '1;
                    fsm = S_NORMAL; // 对于 Uncached load ，需要 refresh 流水线
                end
            end
            S_WAIT_MSTORE: begin
                h_ready = '0;
                c_lsu_req_o.valid = '1;
                c_lsu_req_o.refill_store_req = '1;
                if(c_lsu_resp_i.ready) begin
                    h_ready = '1;
                    l_retire = h_valid_inst_q;
                    l_commit = h_valid_inst_q;
                    fsm = S_NORMAL; // 对于 Uncached load ，需要 refresh 流水线
                end
            end
            S_WAIT_FLUSH: begin
                h_ready = '1;
                l_flush  = '1;
                l_retire = h_valid_inst_q;
                l_commit = '0;
                if(rename_empty_i) begin
                    fsm = S_NORMAL;
                end
            end
        endcase
    end

endmodule
