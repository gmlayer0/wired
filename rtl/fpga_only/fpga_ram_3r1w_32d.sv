// THIS MODULE IS FPGA ONLY
`include "wired0_defines.svh"
module fpga_ram_3r1w_32d#(
    parameter int WIDTH = 32
)( 
    input                          clk,
    input  [4 : 0] addr0,
    input  [4 : 0] addr1,
    input  [4 : 0] addr2,
    input  [4 : 0] addrw,
    output [WIDTH - 1:0]           dout0,
    output [WIDTH - 1:0]           dout1,
    output [WIDTH - 1:0]           dout2,
    input  [WIDTH - 1:0]           din,
    input                          wea
);

for(genvar i = 0 ; i < WIDTH / 2; i++) begin : gen_opram
    qpram_32x2 qpram_inst(
        .CLK(clk),
        .CEN(1'b1), 
        .WEN(wea),
        .A0(addr0),
        .A1(addr1),
        .A2(addr2),
        .AW(addrw),
        .DI(  din[2 * i + 1 : 2 * i]),
        .Q0(dout0[2 * i + 1 : 2 * i]),
        .Q1(dout1[2 * i + 1 : 2 * i]),
        .Q2(dout2[2 * i + 1 : 2 * i])
    );
end
if(WIDTH % 2 != 0) begin : gen_addition_one
    qpram_32x2 qpram_inst(
        .CLK(clk),
        .CEN(1'b1), 
        .WEN(wea),
        .A0(addr0),
        .A1(addr1),
        .A2(addr2),
        .AW(addrw),
        .DI(  din[(2*(WIDTH/2))]),
        .Q0(dout0[(2*(WIDTH/2))]),
        .Q1(dout1[(2*(WIDTH/2))]),
        .Q2(dout2[(2*(WIDTH/2))])
    );
end

endmodule