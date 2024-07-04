`include "wired0_defines.svh"

module wired_skidbuf #(
    parameter type T = logic[31 : 0]
)(
    `_WIRED_GENERAL_DEFINE,
    input  wire inp_valid_i,
    output wire inp_ready_o,
    input  T inp_i,

    output wire oup_valid_o,
    input  wire oup_ready_i,
    output T oup_o
);

    logic skid_busy_q;
    T skid_q;

    always_ff @(posedge clk) if(!skid_busy_q) skid_q <= inp_i;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            skid_busy_q <= '0;
        end else begin
            if(skid_busy_q) skid_busy_q <= !oup_ready_i;
            else skid_busy_q <= inp_valid_i & !oup_ready_i;
        end
    end

    assign oup_o = skid_busy_q ? skid_q : inp_i;
    assign oup_valid_o = inp_valid_i | skid_busy_q;
    assign inp_ready_o = !skid_busy_q;

endmodule
