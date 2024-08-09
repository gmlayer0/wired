# Wired： 对称多核心处理器设计实践

本核心是2024届北京航空航天大学计算机科学与技术学院，由杨建磊老师指导的本科毕设 - 基于 LoongArch 的多核处理器设计与实现 的项目成果。该项目属于北航面向LoongArch国产自主指令集的处理器芯片设计团队毕设的一部分。

## Wired 简介

Wired 是一个由 [gmlayer0](https://github.com/gmlayer0) 设计实现的，基于 Loongarch32 完整指令集的 CPU 核。Wired 核对先进微架构设计及对称多处理器（SMP）设计进行了尝试和探索，成功地实现了乱序双发射且支持硬件缓存一致性的内核微架构。

本核心已验证可以通过 Chiplab（Verilator）进行仿真，并通过 Xilinx Vivado 综合到 FPGA 平台进行验证。

经过 FPGA 验证，可以四核心启动 SMP Linux。

龙芯提供的 LA32R 版 Linux 尚未引入对 SMP 的完整支持。若要启动 SMP Linux，需要使用此[修改版本](https://github.com/gmlayer0/la32r-smp-linux)。

Wired 核心支持浮点指令及所有 LA32 指令，而龙芯教育仅提供了 LA32R 软浮点环境下的编译器。若想发挥本处理器的全部性能，需要自行编译支持 LA32 和单精度硬件浮点的编译器工具链。

Wired 核心整体基于 Tomasulo 算法的思路实现动态调度，采用基于 ROB 的寄存器重命名策略，并实现了分支预测、数据旁路、推测唤醒等机制以提高流水线效率。 Wired 完整实现了 Loongarch32 指令集中的所有整数指令及绝大部分单精度浮点指令，提供单精度浮点执行支持。对于部分特殊单精度浮点指令 (frsqrt.s、fmina.s、fmaxa.s) 没有进行相应的实现，而这部分也不会被编译器生成。

Wired 在龙芯杯提供的性能测试中，可以取得 0.92 的几何平均 IPC ，并在 6 / 10 个测试点中取得 1.0 以上的绝对 IPC。在 CRC32 测试下，处理器核达到了 1.48 的 IPC，在 SHA 测试中则取得了 1.35 的 IPC 。对于分支较稀疏的计算密集程序， Wired 核可以有较为良好的表现。

## 快速上手

### 仿真说明

核心支持使用 Chiplab 进行单核仿真。若要使用 Chiplab 仿真，请参考 [Chiplab官方仓库](https://gitee.com/loongson-edu/chiplab) 完成环境配置，之后设置 `CHIPLAB_HOME` 变量到 chiplab 家目录。

进入 ./src 目录，执行 ./compile_project.py 脚本，即可将本核心自动化部署到 Chiplab 环境之中，之后即可参考官方使用说明运行仿真程序。

### FPGA 综合说明

核心支持使用 Vivado 进行综合，并已在 Ultrascale+ ZYNQ MPSoC(XCZU15EG) 上验证可以以四核心配置启动系统。具体 SoC 及说明请参考 WiredSoC 仓库(尚未公开)。

## 仓库结构说明

Wired 使用 SystemVerilog 实现。所有硬件源码，包括第三方开源 IP，均存放在 ./rtl 目录之下。

而在 ./src 目录之下存放了一些工具脚本，这里对其代码结构进行简单介绍。

### 工具脚本说明

- ./src/compile_project.py

  Wired Core 采用的一套 python 脚本，会读取环境变量中的 `CHIPLAB_HOME` 并将核心部署到其环境中用于仿真。
  其还使用 compile_settings.json 对使用的模块版本进行管理，每个模块的文件头部会用特殊语法标记其版本，并在 compile_settings.json 中实际选择使用的版本。
  描述语法形如：
```
/*--JSON--{"module_name":"core_lsu_rport","module_ver":"2","module_type":"module"}--JSON--*/
```
  这里描述了一个名为  `core_lsu_rport` 的模块，其版本号为 2，类型为模块（module）。

- ./src/gen_sram_wrapper_*.py

  Wired Core 的设计考虑在不同的平台中使用（仿真器环境、FPGA 环境、流片环境）。在不同环境下，核心需要使用不同的模块作为存储器使用。

  好在 Wired Core 设计中只使用了单口及双口的 sram，而 sram 在不同平台下具有相似接近的行为，仅存在模块名和接口名称的区别。

  因此我们使用这样一个小工具，生成我们需要使用的，兼容不同平台，不同容量 sram 的生成工具生成核心使用的 sram。

- ./src/inst
  
  Wired Core 支持使用一套 python 脚本生成核心使用的解码器及相关指令静态控制信息。这套工具使用一组 json 描述指令集。
  
  在 `./src/inst` 目录中，存放默认使用的 la32r 整数指令集解码信息。其中 `general.json` 定义一些常量，以及处理器流水线通用的一些控制信号，及流水级顺序。

  对于常量的定义，直接使用一个键值对即可，由常量名映射到其值。这部分会被使用 `` `define `` 定义成常量使用。
  
  形如：
```json
"_REG_R0_IMM": "2'b11"
```
  等价于 `` `define _REG_R0_IMM 2'b11 ``

  对于流水线的定义，采用一系列二元组，描述流水线关系的 DAG 图，形如：
```json
[
        ["alu", "c_alu_common"],
        ["c", "c_alu_common"],
        ["rob", "c"],
        ["p", "alu"],
        ["p", "rob"],
        ["d", "p"],
        ["Entry", "d"]
]
```

  描述了如下图的一个流水线关系：
![Alt text](https://g.gravizo.com/source/graph_mark000?https%3A%2F%2Fraw.githubusercontent.com%2gmlayer0%2Fwired%2Fmaster%2FREADME.md)
<details> 
<summary></summary>
    graph_mark000
    digraph G {
    start -> d;
    d->p;
    p->alu;
    p->rob;
    rob->c;
    c->c_alu_common;
    alu->c_alu_common;
    c_alu_common->end;
    start [shape=Mdiamond];
    end [shape=Msquare];
    }
graph_mark000
</details>

  关于流水线控制信号定义，也使用一个键值对的格式描述，如：
```json
"imm_type": {
    "length": 3,
    "stage": "is",
    "default_value": "`_IMM_U5"
}
```
  这组定义描述了一个信号名为 `imm_type`，且会在 `is` 级使用（之后流水级不使用）的控制信号。其长度为 3，默认值（不手动指定时）为字符串 `` `_IMM_U5 ``

  控制信号会按照其不同流水线位置，分布在不同流水级之间，并自动生成流水控制信号在各级别之间传递转换的函数。

  在其它 `.json` 文件中，描述了指令使用的具体解码信息，及部分专用指令控制信号，形如 `alu.json` 中对于 `add.w` 指令的定义：
```json
"add.w": {
    "opcode": "00000000000100000",
    "alu_grand_op":"`_ALU_GTYPE_INT",
    "alu_op":"`_ALU_STYPE_ADD",
    "reg_type_r0": "`_REG_RK",
    "reg_type_r1": "`_REG_RJ",
    "reg_type_w": "`_REG_W_RD",
    "alu_inst": 1,
    "das": "\"$r%02x, $r%02x, $r%02x\",rd, rj, rk"
}
```
  其中， opcode 字段定义了指令的解码信息。大部分 LA32R 指令采用可变长的前缀解码的形式，因此只用在 opcode 中描述其前缀码即可。
  对于解码时不关心的位，可以使用 x 跳过，如下指令。
```json
"bcnez": {
    "opcode": "010010xxxxxxxxxxxxxxxx01",
    "fbranch_inst": 1,
    "bcnez": 1,
    "cmp_type": "`_CMP_E",
    "target_type": "`_TARGET_REL",
    "jump_inst": 1,
    "das": "\"%07x\",I26<<2",
    "fpd_inst": 1,
    "slot0": 1
}
```

## TODO

对于分支较为密集且难以预测的排序类测试， Wired 核心尚不支持检查点机制，因此分支预测惩罚高，性能一般。

对于 streamcopy 测试， Wired 核心尚未实现乱序访存及非阻塞缓存，并且对 store 指令的发射逻辑没有进行专门优化。

核心前端取值能力偏弱，无法满足后端需求。

## 第三方 IP 说明

本核心使用了来自 [muntjac](https://github.com/lowRISC/muntjac) 项目的 Tilelink 及 AXI 总线组件、来自 [cvfpu](https://github.com/openhwgroup/cvfpu) 项目的浮点 FPU 部件和 [cva5](https://github.com/openhwgroup/cva5) 项目的部分组件。
