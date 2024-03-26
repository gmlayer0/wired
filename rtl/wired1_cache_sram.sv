// dcache cpu side
`include "wired0_defines"

module wired_cache_sram#(
    parameter integer WORD_SIZE = 32            // support value including {32, 64}
  )(
    `_WIRED_GENERAL_DEFINE,

    input  logic [11:0]               p_addr_i, // CPU 侧访问地址，返回所有路（共四路）上的数据
    output logic [3:0][WORD_SIZE-1:0] p_rdata_o,
    output logic [1:0]                p_sll_o,  // CPU 侧访问需要的偏移（注意：施加在 hit 上，循环左移 rhit = (hit >> p_sll_o) | (hit << (4-p_sll_o)) ）

    input  logic [1:0]  m_way_i,
    input  logic [11:0] m_addr_i,                // 状态机侧访问地址，读写某 way 的整行
    input  logic [3:0][3:0] m_wstrb_i,           // 写掩码，全 0 为读
    input  logic [3:0][31:0] m_wdata_i,
    output logic [3:0][31:0] m_rdata_o,          // 整行，已内部对齐（注：组合逻辑对齐）
  );

  // 四路 SRAM 控制线
  // 为 1024x32b / 512x64b  的双端口类型
  localparam ADDR_LENGTH = $clog2(4096*8/WORD_SIZE)
             logic [3:0][ADDR_LENGTH-1:0] p_sram_addr, m_sram_addr;
  logic [3:0][WORD_SIZE/8-1:0] m_sram_strb;

  logic [3:0][WORD_SIZE-1:0] p_sram_rdata, m_sram_rdata;
  logic [3:0][WORD_SIZE-1:0] m_sram_wdata;
  // 例化 sram
  for(genvar i = 0 ; i < 4 ; i += 1)
  begin : GEN_SRAM
    wire conflict_fire = (|m_sram_strb[i]) && m_sram_addr[i] == p_sram_addr[i];
    reg conflict_fire_q;
    always_ff @(posedge clk)
    begin
      conflict_fire_q <= conflict_fire;
    end
    logic [WORD_SIZE-1:0] p_sram_rdata_raw;
    assign p_sram_rdata[i] = conflict_fire_q ? m_sram_rdata[i] : p_sram_rdata[i];
    wired_dpsram # (
                   .DATA_WIDTH(WORD_SIZE),
                   .DATA_DEPTH(4096*8/WORD_SIZE),
                   .BYTE_SIZE(8)
                 )
                 wired_dpsram_inst (
                   .clk0(clk),
                   .rst_n0(rst_n),
                   // RO Port in verilator
                   .addr0_i(p_sram_addr[i]),
                   .en0_i(!conflict_fire),
                   .we0_i('0),
                   .wdata0_i('0),
                   .rdata0_o(p_sram_rdata_raw),
                   .clk1(clk),
                   .rst_n1(rst_n),
                   .addr1_i(m_sram_addr[i]),
                   .en1_i('1),
                   .we1_i(m_sram_strb[i]),
                   .wdata1_i(m_sram_wdata[i]),
                   .rdata1_o(m_sram_rdata[i])
                 );
  end

  // SRAM 只读端口
  // 注意 SRAM 由于存在一个特别的 BANK 设计
  // |    | B0 | B1 | B2 | B3 |
  // | A0 | W0 | W1 | W2 | W3 |
  // | A1 | W1 | W2 | W3 | W0 |
  // | A2 | W2 | W3 | W0 | W1 |
  // | A3 | W3 | W0 | W1 | W2 |
  // 实际访存的 bank_mask == shuttle(hit >> addr[3:2])
  // 写回/重填时，bank_addr[i][3:2] = way_id - i;
  // 写回：data_128 = {B0, B1, B2, B3} >> (3-way_id);
  // 重填：data_32[i] = data_128[way_id - i];

  // icache 的情况？
  // |    | B0 | B1 | B2 | B3 |
  // | A0 | W0 | W1 | W2 | W3 |
  // |    | W0 | W1 | W2 | W3 |
  // | A2 | W2 | W3 | W0 | W1 |
  // |    | W2 | W3 | W0 | W1 |
  // 实际访存的 bank_mask == shuttle(hit >> addr[3:2])
  // 写回/重填时，bank_addr[i][3:2] = way_id - i;
  // 写回：data_256 = {B0, B1, B2, B3} >> (3-way_id);
  // 重填：data_64[i] = data_256[way_id - i];
  // 即完全一致，只是总线拓宽到 256b
  // 输出逻辑
  // P 口不在这里对齐
  assign p_rdata_o = p_sram_rdata;
  reg  [1:0] p_sll_q;
  always_ff @(posedge clk)
  begin
    p_sll_q <= WORD_SIZE == 64 ? {p_addr_i[3], 1'b0} : {p_addr_i[3:2]};
  end
  assign p_sll_o = p_sll_q;
  // M 口在这里对齐
  reg [1:0] m_way_q;
  always_ff @(posedge clk)
  begin
    m_way_q <= m_way_i;
  end
  if(WORD_SIZE == 32)
  begin
    for(genvar b = 0 ; b < 4 ; b += 1)
    begin
      wire [1:0] m_sel = m_way_q-b[1:0];
      assign m_rdata_o[b] = m_sram_rdata[m_sel];
    end
  end
  else
  begin
    for(genvar b = 0 ; b < 4 ; b += 2)
    begin
      wire [1:0] m_sel = m_way_q-b[1:0];
      assign m_rdata_o[b]   = m_sram_rdata[m_sel][31:0];
      assign m_rdata_o[b+1] = m_sram_rdata[m_sel][63:32];
    end
  end

  // 输入逻辑
  // P 口很直球
  for(genvar b = 0 ; b < 4 ; b += 1)
  begin
    assign p_sram_addr[b] = p_addr_i[11:12-ADDR_LENGTH];
  end
  // M 口要绕一些
  for(genvar b = 0 ; b < 4 ; b += 1)
  begin
    if(WORD_SIZE == 32)
    begin
      wire [1:0] m_sel = m_way_i-b[1:0];
      assign m_sram_addr[b] = {m_addr_i[11:4], m_sel};
      assign m_sram_wdata[b] = m_wdata_i[m_sel];
      assign m_sram_strb[b] = m_wstrb_i[m_sel];
    end
    else
    begin
      wire m_sel = m_way_i[1]-b[1];
      assign m_sram_addr[b] = {m_addr_i[11:4], m_sel};
      assign m_sram_wdata[b][31:0] = m_wdata_i[{m_sel, 1'b0}];
      assign m_sram_wdata[b][63:32] = m_wdata_i[{m_sel, 1'b1}];
      assign m_sram_strb[b][3:0] = m_wstrb_i[{m_sel, 1'b0}] && b[1] == m_way_i[1];
      assign m_sram_strb[b][7:4] = m_wstrb_i[{m_sel, 1'b1}] && b[1] == m_way_i[1];
    end
  end

endmodule
