`include "wired0_defines.svh"

// Fuction module for Wired project
// FPU inorder excution engine
// 顺序完成所有涉及浮点寄存器 fcc 的指令，对于更新立即完成。
// 每次流水线刷新时，立即从提交级获取正确的 fcc 更新到本地

module wired_fcc (
    `_WIRED_GENERAL_DEFINE,

    // 流水线刷新接口
    input  flush_i,
    input  fcc_i, // 正确的 fcc 输入

    // 请求接口
    input  logic  ex_req_valid_i,
    output logic  ex_req_ready_o,
    input  iq_fcc_req_t ex_req_i,

    // 返回端口
    output logic   ex_resp_valid_o,
    input  logic   ex_resp_ready_i,
    output iq_fcc_resp_t ex_resp_o
);

    // 这个模块只需要处理几类指令：
    // movxr2cf:      将数据写入 fcc oknc
    // movcf2xr:      读取 fcc oknc
    // fcmp.cond.s:   修改 fcc oknc
    // bceqz / bcnez: 读取 fcc 判定跳转与否 oknc
    // fsel:          条件赋值 oknc
    // fclass.s       浮点分类指令 oknc

    // 内部维护的 fcc 寄存器
    logic fcc_q, fcc;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            fcc_q <= fcc_i;
        end else begin
            fcc_q <= fcc;
        end
    end

    logic valid_q;
    iq_fcc_req_t req_q;
    always_ff @(posedge clk) if(ex_resp_ready_i) req_q <= ex_req_i;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            valid_q <= '0;
        end else if(ex_resp_ready_i) begin
            valid_q <= ex_req_valid_i;
        end
    end
    assign ex_resp_valid_o = valid_q;
    assign ex_req_ready_o = !valid_q || ex_resp_ready_i;

    // 浮点 Compare 实现的逻辑
    typedef struct packed {
        logic            sign;
        logic [7:0]  exponent;
        logic [22:0] mantissa;
    } fp_t;
    fp_t opa, opb;
    assign opa = req_q.r0;
    assign opb = req_q.r1;

    fpnew_pkg::fp_info_t infoa, infob;
    // Classify input
    fpnew_classifier #(
      .FpFormat    ( fpnew_pkg::FP32 ),
      .NumOperands ( 2               )
      ) i_class_a (
      .operands_i ( {opa,   opb}   ),
      .is_boxed_i ( '1             ),
      .info_o     ( {infoa, infob} )
    );

    wire any_signal = infoa.is_signalling || infob.is_signalling;
    wire any_nan    = infoa.is_nan || infob.is_nan;

    wire op_un = any_nan;
    wire op_eq = !any_nan && (opa == opb || (infoa.is_zero && infob.is_zero));
    wire op_lt = !any_nan && !op_eq && ((opa < opb) ^ (opa.sign || opb.sign));
    wire op_gt = !any_nan && !op_eq && ((opa > opb) ^ (opa.sign || opb.sign));
    wire op_ne = !any_nan && !op_eq;

    wire [4:0] cmp = {op_ne,op_un,op_eq,op_lt,1'd0};

    // CC 变更逻辑
    always_comb begin
        fcc = fcc_q;
        if(req_q.fcmp) begin
            fcc = |(cmp & req_q.cond);
        end
        if(req_q.upd_fcc) begin
            fcc = req_q.r0[0];
        end
    end
    
    // 输出逻辑
    always_comb begin
        ex_resp_o = '0;
        ex_resp_o.need_jump = (req_q.beqz && !fcc_q) || (req_q.bnez && fcc_q);
        ex_resp_o.target_addr = req_q.pc + {{9{req_q.addr_imm[22]}}, req_q.addr_imm[22:0]};
        if(req_q.fsel) begin
            ex_resp_o.result = fcc_q ? opb : opa;
            ex_resp_o.fp_excp.NV = any_signal;
        end else if(req_q.fclass) begin
            ex_resp_o.result[0] = infoa.is_signalling;
            ex_resp_o.result[1] = infoa.is_quiet;
            ex_resp_o.fp_excp.NV = infoa.is_signalling;
            if(opa.sign) begin
                // negative value
                ex_resp_o.result[2]  = infoa.is_inf;
                ex_resp_o.result[3]  = infoa.is_normal;
                ex_resp_o.result[4]  = infoa.is_subnormal;
                ex_resp_o.result[5]  = infoa.is_zero;
            end else begin
                // positive value
                ex_resp_o.result[6]  = infoa.is_inf;
                ex_resp_o.result[7]  = infoa.is_normal;
                ex_resp_o.result[8]  = infoa.is_subnormal;
                ex_resp_o.result[9]  = infoa.is_zero;
            end
        end else begin
            ex_resp_o.result[0] = fcc_q;
        end
        if(req_q.fcmp) begin
            ex_resp_o.fp_excp.NV = any_signal || (any_nan && req_q.cond[0]);
        end
    end

endmodule
