`include "wired0_defines.svh"

module wired_muler (
    `_WIRED_GENERAL_DEFINE,
    input  wire flush_i,

    input  wire valid_i,
    output wire ready_o,
    input  iq_mdu_req_t req_i,

    input  wire ready_i,
    output wire valid_o,
    output iq_mdu_resp_t resp_o
);
    assign ready_o = '1;

    logic inp_valid_q, cal_valid_q, oup_valid_q;
    rob_rid_t inp_wid_q, cal_wid_q, oup_wid_q;
    logic [1:0]  inp_op_q, cal_op_q, oup_op_q;
    logic [32:0] r0_q, r1_q;
    logic [63:0] result_q;
    (* use_dsp = "yes" *) logic[63:0] cal_result_q;
    always_ff @(posedge clk) begin
        if(!rst_n || flush_i) begin
            inp_valid_q <= '0;
            cal_valid_q <= '0;
            oup_valid_q <= '0;
        end else if(ready_i) begin
            inp_valid_q <= valid_i;
            cal_valid_q <= inp_valid_q;
            oup_valid_q <= cal_valid_q;
        end
    end
    always_ff @(posedge clk) begin
        if(ready_i) begin
            r0_q <= {!req_i.op[1] & req_i.r0[31], req_i.r0};
            r1_q <= {!req_i.op[1] & req_i.r1[31], req_i.r1};
            cal_result_q <= $signed(r0_q) * $signed(r1_q);
            result_q <= cal_result_q;
            inp_wid_q <= req_i.wid;
            inp_op_q  <= req_i.op;
            cal_wid_q <= inp_wid_q;
            cal_op_q  <= inp_op_q;
            oup_wid_q <= cal_wid_q;
            oup_op_q  <= cal_op_q;
        end
    end

    assign valid_o = oup_valid_q;
    assign resp_o.wid = oup_wid_q;
    assign resp_o.result = oup_op_q[0] ? result_q[63:32] : result_q[31:0];

endmodule