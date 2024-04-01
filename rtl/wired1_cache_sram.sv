// dcache cpu side
`include "wired0_defines.svh"

module wired_cache_sram#(
    parameter integer WORD_SIZE = 32            // support value including {32, 64}
  )(
    `_WIRED_GENERAL_DEFINE,

    input  logic [11:0]               p_addr_i, // CPU 侧访问地址，返回所有路（共四路）上的数据
    output logic [3:0][WORD_SIZE-1:0] p_rdata_o,
    output cache_tag_t [3:0]          p_rtag_o,

    input  logic [1:0]  m_way_i,
    input  logic [11:0] m_addr_i,                // 状态机侧访问地址，读写某 way 的整行
    input  logic [3:0][3:0] m_wstrb_i,           // 写掩码，全 0 为读
    input  logic [3:0][31:0] m_wdata_i,
    output logic [3:0][31:0] m_rdata_o,          // 整行，已内部对齐（注：组合逻辑对齐）

    input  logic [11:4] t_addr_i,
    input  logic  [3:0] t_we_i,
    input  cache_tag_t  t_wtag_i,
    output cache_tag_t  [3:0] t_rtag_o
  );
  // 四路 TAGSRAM 生成
  for(genvar w = 0 ; w < 4 ; w += 1)
  begin : tag_gen
    cache_tag_t raw_rtag_0, raw_rtag_1; // SRAM 原始输出
    wire conflict = (p_addr_i[11:4] == t_addr_i[11:4]) && t_we_i[w]; // SRAM 输入端口读写冲突
    reg  conflict_q;                                                 // SRAM 输出端口读写冲突
    always_ff @(posedge clk) conflict_q <= conflict;                 // 输入到输出有一周期延迟
    assign p_rtag_o[w] = conflict_q ? raw_rtag_1 : raw_rtag_0; // 端口0（只读）实际输出
    assign t_rtag_o[w] = raw_rtag_1;                           // 端口1（读写）实际输出
    wired_dpsram # (
                   .DATA_WIDTH($bits(cache_tag_t)),
                   .DATA_DEPTH(256),
                   .BYTE_SIZE($bits(cache_tag_t))
                 )
                 wired_dpsram_inst ( // 例化 TAG RAM
                   .clk0(clk),.rst_n0(rst_n),.clk1(clk),.rst_n1(rst_n),
                   // 端口 0，只读
                   .addr0_i(p_addr_i[11:4]),
                   .en0_i(!conflict), // 出现冲突时，不使能
                   .we0_i('0),
                   .wdata0_i('0),
                   .rdata0_o(raw_rtag_0),
                   // 端口 1，读写
                   .addr1_i(t_addr_i[11:4]),
                   .en1_i('1),        // 一直使能
                   .we1_i(t_we_i[w]), // 写使能信号
                   .wdata1_i(t_wtag_i),
                   .rdata1_o(raw_rtag_1)
                 );
  end

  // 四路 SRAM 控制线
  // 为 1024x32b / 512x64b  的双端口类型
  // 对于 P 端口，直接原样输出就行，输入也直连 p_addr_i 就行
  // 对于 M 端口，输出时需要预先做好移位工作
  // 对于每一路的访问地址低位，也需要根据 way 计算。
  logic [3:0][WORD_SIZE-1:0] raw_pdata;
  logic [3:0][WORD_SIZE-1:0] raw_rdata;
  for(genvar b = 0 ; b < 4 ; b += 1)
  begin : data_gen
    // 本地生成 m 口的访问端口
    wire [11:0] m_addr;
    wire [3:0]  m_strb;
    wire [WORD_SIZE/8-1:0] m_strb;
    wire [WORD_SIZE-1:0]   m_wdata;
    if(WORD_SIZE == 32)
    begin
      assign m_addr[11:4] = m_addr_i[11:4];
      assign m_addr[3:2]  = m_way_i - b[1:0];
      assign m_addr[1:0]  = '0;
      assign m_strb  = m_wstrb_i[m_addr[3:2]];
      assign m_wdata = m_wdata_i[m_addr[3:2]];
    end
    else
    begin
      assign m_addr[11:4] = m_addr_i[11:4];
      assign m_addr[3]    = m_way_i[0] ^ b[0];
      assign m_addr[2:0]  = '0;
      assign m_strb[3:0]    = {4{b[1] == m_way_i[1]}} & m_wstrb_i[{m_addr[3],1'b0}];
      assign m_wdata[31:0]  = m_wdata_i[{m_addr[3],1'b0}];
      assign m_strb[7:4]    = {4{b[1] == m_way_i[1]}} & m_wstrb_i[{m_addr[3],1'b1}];
      assign m_wdata[63:32] = m_wdata_i[{m_addr[3],1'b1}];
    end
    wire conflict = p_addr_i[11:$clog2(WORD_SIZE/8)] == m_addr[11:$clog2(WORD_SIZE/8)];
    logic conflict_q;
    always_ff @(posedge clk)
    begin
      conflict_q <= conflict;
    end
    logic [WORD_SIZE-1:0] rdata_p0, rdata_p1;
    assign raw_pdata[b] = conflict_q ? rdata_p1 : rdata_p0;
    assign raw_rdata[b] = rdata_p1;
    wired_dpsram # (
                   .DATA_WIDTH(WORD_SIZE),
                   .DATA_DEPTH(4096*8/WORD_SIZE), // 512 / 1024
                   .BYTE_SIZE(8)
                 )
                 wired_dpsram_inst (
                   .clk0(clk),
                   .rst_n0(rst_n),
                   .addr0_i(p_addr_i[11:$clog2(WORD_SIZE/8)]),
                   .en0_i(!conflict),
                   .we0_i('0),
                   .wdata0_i('0),
                   .rdata0_o(rdata_p0),
                   .clk1(clk),
                   .rst_n1(rst_n),
                   .addr1_i(m_addr[11:$clog2(WORD_SIZE/8)]),
                   .en1_i('1),
                   .we1_i(m_strb),
                   .wdata1_i(m_wdata),
                   .rdata1_o(rdata_p1)
                 );
  end

  // 输出移位
  logic [1:0] m_way_q;
  always_ff @(posedge clk)
  begin
    m_way_q <= m_way_i;
  end
  if(WORD_SIZE == 32)
  begin
    for(genvar s = 0 ; s < 4 ; s += 1)
    begin
      wire [1:0] sft = m_way_q - s[1:0];
      assign m_rdata_o[s] = raw_rdata[sft];
    end
  end
  else
  begin
    for(genvar s = 0 ; s < 4 ; s += 2)
    begin
      wire lr_sel = m_way_q[1];
      wire sft = m_way_q[0] ^ s[1];
      assign m_rdata_o[{s[1], 1'd1}] = raw_rdata[{lr_sel, sft}][63:32];
      assign m_rdata_o[{s[1], 1'd0}] = raw_rdata[{lr_sel, sft}][31:0];
    end
  end
  logic [1:0] p_s_q;
  always_ff @(posedge clk)
  begin
    p_s_q <= p_addr_i[3:2];
  end
  for(genvar w = 0 ; w < 4 ; w += 1)
  begin
    if(WORD_SIZE == 32)
    begin
      wire [1:0] sft = w[1:0] - p_s_q;
      assign p_rdata_o[w] = raw_pdata[sft];
    end
    else
    begin
      assign p_rdata_o[w] = raw_pdata[{w[1], w[0] ^ p_s_q[1]}];
    end
  end

endmodule
