`include "wired0_defines.svh"

module wired_pipereg #(
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

    logic pipe_valid_q;
    T pipe_q;

    always_ff @(posedge clk) if(inp_ready_o) pipe_q <= inp_i;
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            pipe_valid_q <= '0;
        end else begin
            if(inp_ready_o) pipe_valid_q <= inp_valid_i;
        end
    end

    assign oup_o = pipe_q;
    assign oup_valid_o = pipe_valid_q;
    assign inp_ready_o = !pipe_valid_q || oup_ready_i;

endmodule
