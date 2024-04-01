`ifndef _WIRED_MACRO_HEADER
`define _WIRED_MACRO_HEADER

// 此文件中声明了 WIRED 工程将要使用到的全部 macro

// 全局简写使用

// 独热码生成
`define _WIRED_REVERSE(IN, OUT, WIDTH) \
  for(genvar _GEN_I = 0 ; _GEN_I < WIDTH ; _GEN_I+=1) begin : OUT``_REVERSEGEN \
    assign OUT``[_GEN_I] = IN``[WIDTH - 1 - _GEN_I]; \
  end

`define _WIRED_GET_ONEHOT(IN, OUT, WIDTH) \
  assign OUT``[0] = IN``[0]; \
  for(genvar _GEN_I = 1 ; _GEN_I < WIDTH ; _GEN_I+=1) begin : OUT``_ONEHOTGEN \
    assign OUT``[_GEN_I] = (|OUT``[_GEN_I-1:0]) ? '0 : IN``[_GEN_I]; \
  end

`define _WIRED_GET_ONEHOT_REVERSE(IN, OUT, WIDTH) \
  assign OUT``[WIDTH-1] = IN``[WIDTH-1]; \
  for(genvar _GEN_I = WIDTH-2 ; _GEN_I >= 0 ; _GEN_I -= 1) begin : OUT``_ONEHOTGEN \
    assign OUT``[_GEN_I] = (|OUT``[_GEN_I+1:WIDTH-1]) ? '0 : IN``[_GEN_I]; \
  end

// 通用全局时钟及复位信号
`define _WIRED_GENERAL_DEFINE \
input wire clk, \
input wire rst_n

`define _WIRED_GENERAL_CONN \
.clk(clk), \
.rst_n(rst_n)

// 通用全局握手信号定义及接线
`define _WIRED_HANDSHAKE_DEFINE(NAME, T) \
logic NAME``_valid; \
logic NAME``_ready; \
T NAME``_payload;

`define _WIRED_INPORT_DEFINE(NAME, T) \
input  logic NAME``_valid_i, \
output logic NAME``_ready_o, \
input  T NAME``_payload_i

`define _WIRED_INPORT_CONN(PORT_NAME, NAME) \
.PORT_NAME``_valid_i(NAME``_valid), \
.PORT_NAME``_ready_o(NAME``_ready), \
.PORT_NAME``_payload_i(NAME``_payload)

`define _WIRED_INPORT_ASSIGN(PORT_NAME, NAME) \
assign NAME``_valid = PORT_NAME``_valid_i; \
assign PORT_NAME``_ready_o = NAME``_ready; \
assign NAME``_payload = PORT_NAME``_payload_i;

`define _WIRED_OUTPORT_DEFINE(NAME, T) \
output logic NAME``_valid_o, \
input  logic NAME``_ready_i, \
output T NAME``_payload_o

`define _WIRED_OUTPORT_CONN(PORT_NAME, NAME) \
.PORT_NAME``_valid_o(NAME``_valid), \
.PORT_NAME``_ready_i(NAME``_ready), \
.PORT_NAME``_payload_o(NAME``_payload)

`define _WIRED_OUTPORT_ASSIGN(PORT_NAME, NAME) \
assign PORT_NAME``_valid_o = NAME``_valid; \
assign NAME``_ready = PORT_NAME``_ready_i; \
assign PORT_NAME``_payload_o = NAME``_payload;

`define _WIRED_HANDSHAKE_DEFINE_CONN(INNAME, OUTNAME, T) \
`_WIRED_HANDSHAKE_DEFINE(INNAME, T) \
`_WIRED_INPORT_ASSIGN(INNAME, INNAME) \
`_WIRED_HANDSHAKE_DEFINE(OUTNAME, T) \
`_WIRED_OUTPORT_ASSIGN(OUTNAME, OUTNAME)

// 打拍模块生成

// 通用 FF 生成
`define _WIRED_FF(INPUT_NET) \
reg[$bits(INPUT_NET) - 1 : 0] INPUT_NET``_q; \
always_ff @(posedge clk) INPUT_NET``_q <= INPUT_NET;

`define _WIRED_FF_EN(INPUT_NET, EN) \
reg[$bits(INPUT_NET) - 1 : 0] INPUT_NET``_q; \
always_ff @(posedge clk) if(EN) INPUT_NET``_q <= INPUT_NET;

`define _WIRED_FF_RSTABLE(INPUT_NET, RST_VALUE) \
reg[$bits(INPUT_NET) - 1 : 0] INPUT_NET``_q; \
always_ff @(posedge clk) begin \
   if(!rst_n) INPUT_NET``_q <= RST_VALUE; \
   else INPUT_NET``_q <= INPUT_NET; \
end

`define _WIRED_FF_RSTABLE_EN(INPUT_NET, RST_VALUE, EN) \
reg[$bits(INPUT_NET) - 1 : 0] INPUT_NET``_q; \
always_ff @(posedge clk) begin \
   if(!rst_n) INPUT_NET``_q <= RST_VALUE; \
   else if(EN) INPUT_NET``_q <= INPUT_NET; \
end

`endif
