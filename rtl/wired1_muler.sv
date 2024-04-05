module wired_muler #(
    parameter int PKG_SIZE = 32
)(
    `_WIRED_GENERAL_DEFINE,
    input  wire flush_i,

    input  wire valid_i,
    input  wire[1:0] op_i,
    input  wire[31:0] r0_i,
    input  wire[31:0] r1_i,
    input  wire[PKG_SIZE-1:0] pkg_i,

    input  wire ready_i,
    output wire valid_o,
    output wire[PKG_SIZE-1:0] pkg_o,
    output logic[31:0] result_o
);

    // Decode
    // wire signed_ext = !op_i[1]; // _MDU_TYPE_MULH == 3'b001, _MDU_TYPE_MULHU == 3'b011
    // wire get_high = op_i[0];    // _MDU_TYPE_MULH == 3'b001, _MDU_TYPE_MULHU == 3'b011

    logic inp_valid_q, cal_valid_q, oup_valid_q;
    logic[PKG_SIZE-1:0] inp_pkg_q, cal_pkg_q, oup_pkg_q;
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
            r0_q <= {!op_i[1] & r0_i[31], r0_i};
            r1_q <= {!op_i[1] & r1_i[31], r1_i};
            cal_result_q <= $signed(r0_q) * $signed(r1_q);
            result_q <= cal_result_q;
            inp_pkg_q <= pkg_i;
            inp_op_q  <= op_i;
            cal_pkg_q <= inp_pkg_q;
            cal_op_q  <= inp_op_q;
            oup_pkg_q <= cal_pkg_q;
            oup_op_q  <= cal_op_q;
        end
    end

    assign valid_o = oup_valid_q;
    assign pkg_o = oup_pkg_q;
    assign result_o = oup_op_q[0] ? result_q[63:32] : result_q[31:0];

endmodule