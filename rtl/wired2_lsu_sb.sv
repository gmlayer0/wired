// dcache cpu side
`include "wired0_defines.svh"

// 为 lsu 设计的 sb，存在四个表项
module wired_lsu_sb #(
  parameter int SB_SIZE = 4
)(
    `_WIRED_GENERAL_DEFINE,

    // FLUSH 端口
    input  logic flush_i, // 无效化所有表项

    // 写入端口（M1 级）
    output logic ready_o, // 就绪，可以填入新的表项
    input  logic valid_i, // 填入一个表项的请求
    input  sb_meta_t meta_i,

    // 查询端口（M1 级）
    output logic     [SB_SIZE-1:0] valid_o,
    // output logic     [3:0] valid_fwd_o,
    output sb_meta_t [SB_SIZE-1:0] meta_o,
    output logic     [$clog2(SB_SIZE)-1:0] top_o,

    // 提交端口（C 级，顶层命中状态）
    input  logic invalid_i,
    // output logic top_hit_o,
    // output sb_meta_t top_meta_o,

    // SRAM Snoop 端口，实时更新所有 sb 表项
    input  dsram_snoop_t  snoop_i

    // CDB 嗅探端口， store 指令数据未就绪时即可发送到 storebuffer，在 storebuffer中检查。
    ,input pipeline_cdb_t [1:0] cdb_i

  );
  // 内部 FIFO 逻辑
  logic [$clog2(SB_SIZE):0] cnt_q;
  logic [$clog2(SB_SIZE)-1:0] w_ptr_q, r_ptr_q; // w_ptr_q 指向写顶， r_ptr_q 指向读顶
  logic empty_q, full_q;

  always_ff @(posedge clk)
  begin
    if((!rst_n) || flush_i)
    begin
      w_ptr_q <= '0;
      r_ptr_q <= '0;
      empty_q <= '1;
      full_q  <= '0;
      cnt_q   <= '0;
    end
    else
    begin
      if(valid_i && invalid_i)
      begin
        w_ptr_q <= w_ptr_q + 1;
        r_ptr_q <= r_ptr_q + 1;
      end
      else if(valid_i && !invalid_i)
      begin
        w_ptr_q <= w_ptr_q + 1;
        empty_q <= '0;
        full_q  <= cnt_q == (SB_SIZE - 1) ? '1 : '0;
        cnt_q   <= cnt_q + 1;
      end
      else if(!valid_i && invalid_i)
      begin
        r_ptr_q <= r_ptr_q + 1;
        empty_q <= cnt_q == 1 ? '1 : '0;
        full_q  <= '0;
        cnt_q   <= cnt_q - 1;
      end
    end
  end

  // 例化 entrys
  for(genvar i = 0 ; i < SB_SIZE ; i += 1)
  begin
    wire linvalid = ((!rst_n) || flush_i) || (invalid_i && r_ptr_q == i[$clog2(SB_SIZE)-1:0]);
    wire lvalid = valid_i && w_ptr_q == i[$clog2(SB_SIZE)-1:0];
    wired_lsu_sb_entry  wired_lsu_sb_entry_inst (
                          `_WIRED_GENERAL_CONN,
                          .invalid_i(linvalid),
                          .valid_i(lvalid),
                          .meta_i(meta_i),
                          .valid_o(valid_o[i]),

                          .gvalid_i(valid_i),
                          // .valid_fwd_o(valid_fwd_o[i]),
                          .meta_o(meta_o[i]),
                          .snoop_i(snoop_i)
                        );
  end

  // 输出逻辑
  assign top_o = r_ptr_q;
  assign ready_o = !full_q;
  // assign top_meta_o = meta_o[r_ptr_q];
  // assign top_hit_o = |(meta_o[r_ptr_q].hit);

endmodule
