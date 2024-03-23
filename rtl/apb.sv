module mmio_device(
    input PCLK,    // 时钟
    input PRESETn, // 复位

    input PVALID,  // 请求有效
    input PADDR,   // 请求属性：地址
    input PWRITE,  // 请求属性：读写
    input PWDATA,  // 请求属性：写数据

    output reg PREADY, // 请求得到响应
    output reg PRDATA,  // 返回属性：读数据

    output [m-1:0] M_PVALID,  // 请求有效
    output [m-1:0] M_PADDR,   // 请求属性：地址
    output [m-1:0] M_PWRITE,  // 请求属性：读写
    output [m-1:0] M_PWDATA,  // 请求属性：写数据

    input  [m-1:0] M_PREADY,
    input  [m-1:0] M_PRDATA
);

    reg csr_q;
    always @(posedge clk) begin
        if(~PRESETn) begin
            // 复位逻辑
            csr_q <= 1'b0; // 初始值
            PREADY <= 1'b0;
        end else begin
            if(PVALID & !PREADY) begin
                PREADY <= 1'b1;
                if(PWRITE) begin
                    // 设备执行 write(PADDR, PWDATA); 逻辑
                end else begin
                    // 设备执行 PRDATA = read(PADDR); 逻辑
                end
            end else begin
                PREADY <= 1'b0;
            end
        end
    end

endmodule
