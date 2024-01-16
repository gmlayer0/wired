# Project WIRED

## About "Layer"

- Layer 0 is for all header files and portable file
- Layer 1 is for all general modules
- Layer 2 is for all functional modules
- Layer 3 is for frontend/backend top level
- Layer 4 is for core-top level

## About plan and timing
计划上，先对后端进行实现再处理前端，早期前端可以使用 fake 版本，直接抓取内存。

- 1.16 计划完成重命名级别（前置要求，多口 Register file 用以存储重命名表格）
- 1.17 计划完成 ROB 设计
