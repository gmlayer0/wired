// dcache cpu side
`include "wired0_defines.svh"

// 为 lsu 设计的 sb 单个表项实现
// fun fact: this module does not need to be reset actually.
module wired_lsu_sb_entry(
    `_WIRED_GENERAL_DEFINE,

    // 写入端口
    input  logic        invalid_i, // 表项提交
    input  logic        valid_i,   // 表项创建
    
    input  logic        gvalid_i,  // 全局表项创建（监听 strb 变化情况）
    input  sb_meta_t    meta_i,
    
    // 查询端口
    output logic        valid_o,
    // output logic        valid_fwd_o,
    output sb_meta_t    meta_o,

    // SRAM Snoop 端口，实时更新所有 sb 表项
    input  dsram_snoop_t  snoop_i

  );

  // 表项有效性追踪
  logic valid_q;
  assign valid_o = valid_q;
  // assign valid_fwd_o = !invalid_i && valid_q;
  always_ff @(posedge clk) begin
    if(invalid_i) begin
      valid_q <= '0;
    end else if(valid_i) begin // 这两个其实不可能同时发生
      valid_q <= '1;
    end
  end

  // 表项元数据追踪
  sb_meta_t meta;
  sb_meta_t meta_q;
  assign meta_o = meta_q;
  always_ff @(posedge clk) begin
    if(valid_i) begin
      // 这里不用做 snoop，因为外面已经做过了 ^_^
      meta_q <= meta_i;
    end else begin
      // snoop 更新 meta_q
      meta_q <= meta;
    end
  end
  always_comb begin
    meta = meta_q;
    if(meta_q.paddr[11:4] == snoop_i.taddr[11:4]) begin
      for(integer w = 0 ; w < 4 ; w += 1) begin
        if(snoop_i.twe[w]) begin
          // 对应路被写入，执行更新
          meta.hit[w] = (snoop_i.t.p == meta_q.paddr[31:12]) && snoop_i.t.wp; // 可写且命中
        end
      end
    end
    if(gvalid_i && meta_q.paddr[31:2] == meta_i.paddr[31:2]) begin
      // 对同一个字的写入，无效化最新的
      meta.fwd_strb = meta_q.fwd_strb & ~meta_i.fwd_strb;
    end
  end

endmodule
