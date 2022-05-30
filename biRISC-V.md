# biRISC-V

###### tags: `Computer Architecture`

:::warning
## Expectation (Goal)
1. biRISC 和課堂提到的 RISC-V 處理器實作很不同， 本質是 superscalar (dual-issue) in-order 6 or 7 stage pipeline，是進階的處理器實作，但在本專題中，你們只要專注在 biRISC 的 datapath, control, pipeline 的內部設計
2. 使用 Verilator 工具可用來確認 biRISC 的 RV32IMZicsr 指令的 datapath 和 control
3. 探討 6 or 7 stage pipeline 的設計，搭配分析對應的波形圖，特別是解讀如何達到 dual-issue
4. 在共筆記錄你們的探索過程，及相關問題
:::
## Environment Setup
1. Cloning: To clone this project and its dependencies
   > git clone --recursive https://github.com/ultraembedded/biriscv.git

2. Running HelloWorld (For testing)
   1. Install Icarus Verilog (Debian / Ubuntu / Linux Mint)
      > sudo apt-get install iverilog
   2. Run a simple test image (test.elf)
      > cd tb/tb_core_icarus
      > make
      
      - result
      > makefile:9: *** riscv32-unknown-elf-objcopy missing from PATH.  Stop.
   
   To solve this problem, We modify the 7 line of the Makefile (in ./tb/tb_core_icarus).
   > OBJCOPY ?= riscv32-unknown-elf-objcopy &rarr; OBJCOPY ?= riscv-none-embed-objcopy
   
   **make** again!
   
   - result
   ```shell=
    vvp build//output.out -vcd
    Starting bench
    VCD info: dumpfile waveform.vcd opened for output.

    Test:
    1. Initialised data
    2. Multiply
    3. Divide
    4. Shift left
    5. Shift right
    6. Shift right arithmetic
    7. Signed comparision
    8. Word access
    9. Byte access
    10. Comparision

   ```
   
## biRISC-V introduction - 32-bit dual issue RISC-V CPU
In [biRISC-V](https://github.com/ultraembedded/biriscv), We can find that there are seven stages in biRISC-V.
&rarr; `PC`, `Fetch`, `Pre-decode`, `Issue`, `ALU`, `Mem` and `Writeback`, respectively.
![](https://i.imgur.com/DDh8wwf.png)

- biRISC-V Feature
  - 32-bit RISC-V ISA CPU core.
  - Superscalar (dual-issue) in-order 6 or 7 stage pipeline.
  - Support RISC-V’s integer (I), multiplication and division (M), and CSR instructions (Z) extensions (RV32IMZicsr).
  - Branch prediction (bimodel/gshare) with configurable depth branch target buffer (BTB) and return address stack (RAS).
  - 64-bit instruction fetch, 32-bit data access.
  - 2 x integer ALU (arithmetic, shifters and branch units).
  - 1 x load store unit, 1 x out-of-pipeline divider.
  - Issue and complete up to 2 independent instructions per cycle.
  - Supports user, supervisor and machine mode privilege levels.
  - Basic MMU support - capable of booting Linux with atomics (RV-A) SW emulation.
  - Implements base ISA spec v2.1 and privileged ISA spec v1.11.
  - Verified using Google's RISCV-DV random instruction sequences using cosimulation against C++ ISA model.
  - Support for instruction / data cache, AXI bus interfaces or tightly coupled memories.
  - Configurable number of pipeline stages, result forwarding options, and branch prediction resources.
  - Synthesizable Verilog 2001, Verilator and FPGA friendly.
  - Coremark: 4.1 CoreMark/MHz
  - Dhrystone: 1.9 DMIPS/MHz ('legal compile options' / 337 instructions per iteration)

- How to analyze
  In the beginning, We use the test.elf in the folder **/src/tb/tb_core_icaris** to analyze biRISC-V core.
  From the 29 line instruction of Makefile, we can find out what verilog file we will use. 
  > SRC_V ?= $(foreach src,$(SRC_V_DIR),$(wildcard $(src)/*.v)
  :::spoiler the verilog files will use
  ```shell=
    biriscv_pipe_ctrl.v 
    biriscv_alu.v
    biriscv_xilinx_2r1w.v 
    biriscv_issue.v 
    biriscv_mmu.v 
    biriscv_decoder.v 
    biriscv_frontend.v 
    biriscv_exec.v 
    biriscv_lsu.v 
    biriscv_decode.v 
    biriscv_trace_sim.v 
    biriscv_multiplier.v 
    biriscv_regfile.v 
    biriscv_fetch.v 
    biriscv_npc.v 
    biriscv_defs.v 
    biriscv_csr.v 
    biriscv_csr_regfile.v 
    biriscv_divider.v 
    riscv_core.v 
    tcm_mem.v 
    tb_top.v 
    tcm_mem_ram.v
  ```
  :::
  
  According these files, we can find out how biRISC-V CPU work, and next step we will analyze the datapath, control, and pipeline of biRISC-V.
  
  To analyze the biRISC-V, we use the test.elf (in ./tb/tb_core_icarus) as a example.
  The disasembly of this file is below.
  > riscv-none-embed-objdump -d test.elf > test.s
    
  ![](https://i.imgur.com/097w6Vq.png)

## Requirement 1, biRISC-V pipeline
### PC Stage
To understand how pc(program counter) work, We trace the code in `biriscv_npc.v`, and there are some interest discoveries.

- Control
  The following lists are the control parameters and the inputs/output of the PC stage.
  
  In parameters, we can find some design, like [Branch Predictor(BTB and BHT)](https://users.ece.cmu.edu/~jhoe/course/ece447/S21handouts/L10.pdf) which is used in biriscv. 
  
  Simultaneously, user can enable [GSHARE(Global branch predictor)](https://people.engr.ncsu.edu/efg/521/f02/common/lectures/notes/lec16.pdf). 
  ```verilog=
     parameter SUPPORT_BRANCH_PREDICTION = 1 /* Enable branch prediction structures. */
    ,parameter NUM_BTB_ENTRIES  = 32         /* Number of branch target buffer entries. */
    ,parameter NUM_BTB_ENTRIES_W = 5         /* Set to log2(NUM_BTB_ENTRIES). */
    ,parameter NUM_BHT_ENTRIES  = 512        /* Number of branch history table entries. */
    ,parameter NUM_BHT_ENTRIES_W = 9         /* Set to log2(NUM_BHT_ENTRIES_W). */
    ,parameter RAS_ENABLE       = 1          /* Enable branch history table based prediction. */
    ,parameter GSHARE_ENABLE    = 0          /* Enable GSHARE branch prediction algorithm. */
    ,parameter BHT_ENABLE       = 1          /* Enable branch history table based prediction. */
    ,parameter NUM_RAS_ENTRIES  = 8          /* Number of return stack addresses supported. */
    ,parameter NUM_RAS_ENTRIES_W = 3         /* Set to log2(NUM_RAS_ENTRIES_W). */
  ```
  
  In I/O, there are some system signals(clk_i and rst_i ...) and branch signals(branch_request_i and branch_is_taken_i ...).
  
  Also, if occurred the branch condition, we could know what the instruction is, like **ret** (using branch_is_ret_i) or **j** (branch_is_jmp_i).
  ```verilog=
    // Inputs
     input           clk_i    /* Clock input */
    ,input           rst_i    /* Asynchronous reset, active-high. Reset memory / AXI interface. */
    ,input           invalidate_i
    ,input           branch_request_i
    ,input           branch_is_taken_i        /* branch is taken or not */
    ,input           branch_is_not_taken_i    /* branch is not taken or not */
    ,input  [ 31:0]  branch_source_i
    ,input           branch_is_call_i
    ,input           branch_is_ret_i    /* branch is ret or not */ 
    ,input           branch_is_jmp_i    /* branch is j or not */
    ,input  [ 31:0]  branch_pc_i
    ,input  [ 31:0]  pc_f_i
    ,input           pc_accept_i
    
    // Outputs
    ,output [ 31:0]  next_pc_f_o
    ,output [  1:0]  next_taken_f_o
  ```
  
  From I/O, we can plot a simple graph.
  ![](https://i.imgur.com/vDFamDI.png)
  
- Datapath
  In datapath, from the following source code, it is divide into two parts, `BRANCH_PREDICTION` and `NO_BRANCH_PREDICTION`, respectively.
  ```verilog=
    generate
    if (SUPPORT_BRANCH_PREDICTION)
    begin: BRANCH_PREDICTION
        ...
    end
    else
    begin: NO_BRANCH_PREDICTION
        ...
    end
    endgenerate
  ```
  
  The following source code shows if `SUPPORT_BRANCH_PREDICTION = 1`, how to compute the output `next_pc_f_o` and `next_taken_f_o`.
  ```verilog=
    assign btb_valid_w   = btb_valid_r;
    assign btb_upper_w   = btb_upper_r;
    assign btb_is_call_w = btb_is_call_r;
    assign btb_is_ret_w  = btb_is_ret_r;
    assign next_pc_f_o   = ras_ret_pred_w      ? ras_pc_pred_w : 
                           (bht_predict_taken_w | btb_is_jmp_r) ? btb_next_pc_r :
                           {pc_f_i[31:3],3'b0} + 32'd8;

    assign next_taken_f_o = (btb_valid_w & (ras_ret_pred_w | bht_predict_taken_w | btb_is_jmp_r)) ? 
                            pc_f_i[2] ? {btb_upper_r, 1'b0} :
                            {btb_upper_r, ~btb_upper_r} : 2'b0;

    assign pred_taken_w   = btb_valid_w & (ras_ret_pred_w | bht_predict_taken_w | btb_is_jmp_r) & pc_accept_i;
    assign pred_ntaken_w  = btb_valid_w & ~pred_taken_w & pc_accept_i;
        
  ```
  
  The following source code shows the condition of `SUPPORT_BRANCH_PREDICTION = 0`.
  ```verilog=
    assign next_pc_f_o    = {pc_f_i[31:3],3'b0} + 32'd8;
    assign next_taken_f_o = 2'b0;
  ```
  
### Fetch Stage
- Control
  In file `biriscv_fetch.v`, there is only one parameter to setup — `SUPPORT_MMU`
  ```verilog=
    parameter SUPPORT_MMU      = 1 /* Enable basic memory management unit. */
  ```
  
  The following source code from `biriscv_fetch.v` is the I/O of the fetch stage. We can know that the birisc-v get instruction from icache, and there are some control bits of cache, like valid, accept and error...
  :::spoiler I/O 
  ```verilog=
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           fetch_accept_i
    ,input           icache_accept_i
    ,input           icache_valid_i    
    ,input           icache_error_i
    ,input  [ 63:0]  icache_inst_i
    ,input           icache_page_fault_i    /* Is page fault or not */
    ,input           fetch_invalidate_i
    ,input           branch_request_i
    ,input  [ 31:0]  branch_pc_i
    ,input  [  1:0]  branch_priv_i
    ,input  [ 31:0]  next_pc_f_i
    ,input  [  1:0]  next_taken_f_i

    // Outputs
    ,output          fetch_valid_o
    ,output [ 63:0]  fetch_instr_o    /* The instruction get from icache */
    ,output [  1:0]  fetch_pred_branch_o
    ,output          fetch_fault_fetch_o
    ,output          fetch_fault_page_o
    ,output [ 31:0]  fetch_pc_o
    ,output          icache_rd_o
    ,output          icache_flush_o
    ,output          icache_invalidate_o
    ,output [ 31:0]  icache_pc_o
    ,output [  1:0]  icache_priv_o
    ,output [ 31:0]  pc_f_o
    ,output          pc_accept_o

  ```
  :::
  
- Datapath
  In datapath, we find a special design called [Skid Buffer](http://fpgacpu.ca/fpga/Pipeline_Skid_Buffer.html).
  ```
  Input                       Output
  -----                       ------
            -------------
  ready <--|             |<-- ready
  valid -->| Skid Buffer |--> valid
  data  -->|             |--> data
            -------------
  ```
  
  The following source code is the implementation of Skid Buffer, and we can know how biRISC-V get instruction.
  ```verilog=
    reg [99:0]  skid_buffer_q;
    reg         skid_valid_q;

    always @ (posedge clk_i or posedge rst_i)
    if (rst_i)
    begin
        skid_buffer_q  <= 100'b0;
        skid_valid_q   <= 1'b0;
    end 
    // Instruction output back-pressured - hold in skid buffer
    else if (fetch_valid_o && !fetch_accept_i)
    begin
        skid_valid_q  <= 1'b1;
        skid_buffer_q <= {fetch_fault_page_o, fetch_fault_fetch_o, fetch_pred_branch_o, fetch_pc_o, fetch_instr_o};
    end
    else
    begin
        skid_valid_q  <= 1'b0;
        skid_buffer_q <= 100'b0;
    end

    assign fetch_valid_o       = (icache_valid_i || skid_valid_q) & !fetch_resp_drop_w;
    assign fetch_pc_o          = skid_valid_q ? skid_buffer_q[95:64] : {pc_d_q[31:3],3'b0};
    assign fetch_instr_o       = skid_valid_q ? skid_buffer_q[63:0]  : icache_inst_i;
    assign fetch_pred_branch_o = skid_valid_q ? skid_buffer_q[97:96] : pred_d_q;

    // Faults
    assign fetch_fault_fetch_o = skid_valid_q ? skid_buffer_q[98] : icache_error_i;
    assign fetch_fault_page_o  = skid_valid_q ? skid_buffer_q[99] : icache_page_fault_i;

    assign pc_f_o              = icache_pc_w;
    assign pc_accept_o         = ~stall_w;
  ```

### Pre-decode Stage

- Control
  In decode stage, decoder get the instruction and identify the operation by opcode, then send it to `fetch_fifo`, which is a queue to distribute the instruction to `decoder0` and `decoder1`

#### Analyze by source code
we need to see the definitions in `biriscv_defs.v`, and there are the instruction definitions which is shown as following below
:::spoiler Definition of Instruction and Instruction Mask
```verilog=
//--------------------------------------------------------------------
// Instructions Masks
//--------------------------------------------------------------------
// andi
`define INST_ANDI 32'h7013
`define INST_ANDI_MASK 32'h707f

// addi
`define INST_ADDI 32'h13
`define INST_ADDI_MASK 32'h707f

// slti
`define INST_SLTI 32'h2013
`define INST_SLTI_MASK 32'h707f

// sltiu
`define INST_SLTIU 32'h3013
`define INST_SLTIU_MASK 32'h707f

// ori
`define INST_ORI 32'h6013
`define INST_ORI_MASK 32'h707f

// xori
`define INST_XORI 32'h4013
`define INST_XORI_MASK 32'h707f

// slli
`define INST_SLLI 32'h1013
`define INST_SLLI_MASK 32'hfc00707f

// srli
`define INST_SRLI 32'h5013
`define INST_SRLI_MASK 32'hfc00707f

// srai
`define INST_SRAI 32'h40005013
`define INST_SRAI_MASK 32'hfc00707f

// lui
`define INST_LUI 32'h37
`define INST_LUI_MASK 32'h7f

// auipc
`define INST_AUIPC 32'h17
`define INST_AUIPC_MASK 32'h7f

// add
`define INST_ADD 32'h33
`define INST_ADD_MASK 32'hfe00707f

// sub
`define INST_SUB 32'h40000033
`define INST_SUB_MASK 32'hfe00707f

// slt
`define INST_SLT 32'h2033
`define INST_SLT_MASK 32'hfe00707f

// sltu
`define INST_SLTU 32'h3033
`define INST_SLTU_MASK 32'hfe00707f

// xor
`define INST_XOR 32'h4033
`define INST_XOR_MASK 32'hfe00707f

// or
`define INST_OR 32'h6033
`define INST_OR_MASK 32'hfe00707f

// and
`define INST_AND 32'h7033
`define INST_AND_MASK 32'hfe00707f

// sll
`define INST_SLL 32'h1033
`define INST_SLL_MASK 32'hfe00707f

// srl
`define INST_SRL 32'h5033
`define INST_SRL_MASK 32'hfe00707f

// sra
`define INST_SRA 32'h40005033
`define INST_SRA_MASK 32'hfe00707f

// jal
`define INST_JAL 32'h6f
`define INST_JAL_MASK 32'h7f

// jalr
`define INST_JALR 32'h67
`define INST_JALR_MASK 32'h707f

// beq
`define INST_BEQ 32'h63
`define INST_BEQ_MASK 32'h707f

// bne
`define INST_BNE 32'h1063
`define INST_BNE_MASK 32'h707f

// blt
`define INST_BLT 32'h4063
`define INST_BLT_MASK 32'h707f

// bge
`define INST_BGE 32'h5063
`define INST_BGE_MASK 32'h707f

// bltu
`define INST_BLTU 32'h6063
`define INST_BLTU_MASK 32'h707f

...
:::

The module structure in decode stage
```Tree=
biriscv_decode
    ├──biriscv_decoder
    └──fetch_fifo
```
So we need to know these three module which is in `biriscv_decode.v` and `biriscv_decoder.v`

1. `biriscv_decoder` 
   This module here is to decode the instrution by opcode, analyze which type instruction is, and which operation the processor need to takes
   :::spoiler I/O ports
   ```verilog=
    module biriscv_decoder
    (
         input             valid_i            // boolean for valid instruction
        ,input             fetch_fault_i      // maybe the fault is page fault or instruction fault
        ,input             enable_muldiv_i    // boolean for multiple instruction or division instruction
        ,input  [31:0]     opcode_i           // instruction opcode           

        ,output            invalid_o          
        ,output            exec_o             // logical and shift operation
        ,output            lsu_o              // load and store operation
        ,output            branch_o           // branch operation
        ,output            mul_o              // multiplied operation
        ,output            div_o              // divided operation
        ,output            csr_o              // system call, exception, interrupt operation 
        ,output            rd_valid_o
    );
    ```
    :::
    
2. `fetch_fifo`
    - This is a instruction prefetch queue
    - Parameters:
    ```verilog=
    #(
        parameter WIDTH   = 64,               // Instruction Size
        parameter DEPTH   = 2,                
        parameter ADDR_W  = 1,
        parameter OPC_INFO_W = 10             // Opcode size
    )
    ```
    :::spoiler I/O Ports:
    ```verilog=
    (
     input                  clk_i           // clock
    ,input                  rst_i           // reset
    ,input                  flush_i         // boolean for flush the instruction

    // Input side
    ,input                  push_i         
    ,input  [31:0]          pc_in_i          
    ,input  [1:0]           pred_in_i
    ,input  [WIDTH-1:0]     data_in_i
    ,input [OPC_INFO_W-1:0] info0_in_i      
    ,input [OPC_INFO_W-1:0] info1_in_i       
    ,output                 accept_o

    // Outputs, here are two decoder
    // decoder 0
    ,output                 valid0_o
    ,output  [31:0]         pc0_out_o       // program counter the decoder pop out
    ,output [(WIDTH/2)-1:0] data0_out_o     
    ,output[OPC_INFO_W-1:0] info0_out_o
    ,input                  pop0_i          // pop to thread 0 from the fetch_fifo
    
    // decoder 1
    ,output                 valid1_o
    ,output  [31:0]         pc1_out_o
    ,output [(WIDTH/2)-1:0] data1_out_o
    ,output[OPC_INFO_W-1:0] info1_out_o
    ,input                  pop1_i
    );

    ```
    :::
    
    - Here is how the buffer distribute the opcode to `decoder 0` and `decoder 1`
    ```verilog=
    if (rst_i)
    begin
        count_q   <= {(COUNT_W) {1'b0}};
        rd_ptr_q  <= {(ADDR_W) {1'b0}};
        wr_ptr_q  <= {(ADDR_W) {1'b0}};

        for (i = 0; i < DEPTH; i = i + 1) 
        begin
            ram_q[i]         <= {(WIDTH) {1'b0}};
            pc_q[i]          <= 32'b0;
            info0_q[i]       <= {(OPC_INFO_W) {1'b0}};
            info1_q[i]       <= {(OPC_INFO_W) {1'b0}};
            valid0_q[i]      <= 1'b0;
            valid1_q[i]      <= 1'b0;
        end
    end
    else if (flush_i)
    begin
        count_q   <= {(COUNT_W) {1'b0}};
        rd_ptr_q  <= {(ADDR_W) {1'b0}};
        wr_ptr_q  <= {(ADDR_W) {1'b0}};

        for (i = 0; i < DEPTH; i = i + 1) 
        begin
            info0_q[i]       <= {(OPC_INFO_W) {1'b0}};
            info1_q[i]       <= {(OPC_INFO_W) {1'b0}};
        end
    end
    ```
    then concatenate the last 3 bit to the previous instruction
    ```verilog=
    assign pc0_out_o     = {pc_q[rd_ptr_q][31:3],3'b000};
    assign pc1_out_o     = {pc_q[rd_ptr_q][31:3],3'b100};
    ```
    Due to above code, there is the reason why PC of decoder1 and PC of decoder 2 have address difference of 4
    
3. `biriscv_decode` which is combined with `biriscv_decoder` and `fetch_fifo`
    :::spoiler I/O Ports:
    ```verilog=
    (
        // Inputs
         input           clk_i
        ,input           rst_i
        ,input           fetch_in_valid_i
        ,input  [ 63:0]  fetch_in_instr_i
        ,input  [  1:0]  fetch_in_pred_branch_i
        ,input           fetch_in_fault_fetch_i
        ,input           fetch_in_fault_page_i
        ,input  [ 31:0]  fetch_in_pc_i
        ,input           fetch_out0_accept_i
        ,input           fetch_out1_accept_i
        ,input           branch_request_i
        ,input  [ 31:0]  branch_pc_i
        ,input  [  1:0]  branch_priv_i

        // Outputs: Operation this instruction need to do, and the decoder will send to corresponding hardware by these value
        ,output          fetch_in_accept_o
        ,output          fetch_out0_valid_o
        ,output [ 31:0]  fetch_out0_instr_o
        ,output [ 31:0]  fetch_out0_pc_o
        ,output          fetch_out0_fault_fetch_o
        ,output          fetch_out0_fault_page_o
        ,output          fetch_out0_instr_exec_o
        ,output          fetch_out0_instr_lsu_o
        ,output          fetch_out0_instr_branch_o
        ,output          fetch_out0_instr_mul_o
        ,output          fetch_out0_instr_div_o
        ,output          fetch_out0_instr_csr_o
        ,output          fetch_out0_instr_rd_valid_o
        ,output          fetch_out0_instr_invalid_o
        ...
    );
    ```
    :::
### Issue Stage
There are three units in this Stage, Controller, Divider and Register File.

#### Register file
- This register file just modified traditional 5-stages pipeline. Here is no write back siganl, and we can read more register value for two pipelines.
- Data flow in register files
  ![](https://i.imgur.com/E86zPm9.png)
:::spoiler I/O Ports
```verilog=
(
    // Inputs
     input           clk_i       // clock
    ,input           rst_i       // reset
    ,input  [  4:0]  rd0_i       // thread 0 destination register
    ,input  [  4:0]  rd1_i
    ,input  [ 31:0]  rd0_value_i 
    ,input  [ 31:0]  rd1_value_i
    ,input  [  4:0]  ra0_i
    ,input  [  4:0]  rb0_i
    ,input  [  4:0]  ra1_i
    ,input  [  4:0]  rb1_i

    // Outputs: 4 register 
    ,output [ 31:0]  ra0_value_o // value of thread 0 register a
    ,output [ 31:0]  rb0_value_o // value of thread 0 register b
    ,output [ 31:0]  ra1_value_o
    ,output [ 31:0]  rb1_value_o
);
```
:::
#### DIV
- Control
  The following source code is the I/O of divider in biRISC-V. It contains the signals which has decoded by decoder, like rd, rs1, rs2 and opcode ... .
  
  :::spoiler I/O Ports
  ```verilog=
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input           opcode_valid_i
    ,input  [ 31:0]  opcode_opcode_i
    ,input  [ 31:0]  opcode_pc_i
    ,input           opcode_invalid_i
    ,input  [  4:0]  opcode_rd_idx_i    /* rd index */
    ,input  [  4:0]  opcode_ra_idx_i    /* rs1 index */
    ,input  [  4:0]  opcode_rb_idx_i    /* rs2 index */
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i

    // Outputs
    ,output          writeback_valid_o
    ,output [ 31:0]  writeback_value_o  /* return value */
  ```
  :::
  
  ![](https://i.imgur.com/OnBWEBE.png)

- Datapath
  The following source code is the definition of `div` and `rem` instruction and mask.
  ```verilog=
    // mulhu
    `define INST_MULHU 32'h2003033
    `define INST_MULHU_MASK 32'hfe00707f

    // div
    `define INST_DIV 32'h2004033
    `define INST_DIV_MASK 32'hfe00707f

    // divu
    `define INST_DIVU 32'h2005033
    `define INST_DIVU_MASK 32'hfe00707f

    // rem
    `define INST_REM 32'h2006033
    `define INST_REM_MASK 32'hfe00707f

    // remu
    `define INST_REMU 32'h2007033
    `define INST_REMU_MASK 32'hfe00707f
  ```
  
  The code below shows how biRISC-V identifies the `div` instruction in divider.
  
  ```verilog=
    wire inst_div_w         = (opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV;
    wire inst_divu_w        = (opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU;
    wire inst_rem_w         = (opcode_opcode_i & `INST_REM_MASK) == `INST_REM;
    wire inst_remu_w        = (opcode_opcode_i & `INST_REMU_MASK) == `INST_REMU;

    wire div_rem_inst_w     = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV)  || 
                              ((opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU) ||
                              ((opcode_opcode_i & `INST_REM_MASK) == `INST_REM)  ||
                              ((opcode_opcode_i & `INST_REMU_MASK) == `INST_REMU);

    wire signed_operation_w = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV) || ((opcode_opcode_i & `INST_REM_MASK) == `INST_REM);
    wire div_operation_w    = ((opcode_opcode_i & `INST_DIV_MASK) == `INST_DIV) || ((opcode_opcode_i & `INST_DIVU_MASK) == `INST_DIVU);

  ```
  
  The following code is the divider output, `writeback_value_o` means the result of divider, and the result will be writeback to E1 stage.
  ```verilog=
    assign writeback_valid_o = valid_q;
    assign writeback_value_o  = wb_result_q;
  ```
#### Pipeline Control
Here we will analyze the controller of `biRISC-V`, Controller is the most important part in Pipeline. It needs to detect hazard, issue and schedule, track instruction status and branch prediction.

The parameters here are supporting the dual issue, multiplier and divider, load bypass and multiply bypass
```verilog=
#(
     parameter SUPPORT_MULDIV   = 1     
    ,parameter SUPPORT_DUAL_ISSUE = 1
    ,parameter SUPPORT_LOAD_BYPASS = 1
    ,parameter SUPPORT_MUL_BYPASS = 1
    ,parameter SUPPORT_REGFILE_XILINX = 0
)
```

- There are the cycles for each operation:
    - `ALU`: 1
    - `MUL`: 2~3
    - `Load Store`: 2~3
    - `DIV`: 2~34
    - `CSR`: 2~3
- Issue and scheduling:
  - Check the instruction can be issued in the second thread
  - Will not issue `MUL`, `DIV`, `CSR` after `load`, can only issue `ALU` operation
  - Stalling or not, if stalling &rarr; no issue
  - Check the execution units for latency:
    - if no bypassing, load and multiply will take at least 2 latency
    - if bypassing, load and multiply will take at least 1 latency
  - Slot 0 issue `Branch`, `LSU`, `ALU`, `MUL`, `DIV` and `CSR`
  - Slot 1 issue `Branch`, `LSU`, `ALU` and `MUL`
    
- Branch Prediction: This information is used to learn future prediction, and to correct `BTB`, `BHT`, `GShare`, `RAS` indexes on mispredictions. Link to `PC` Stage.
    ```verilog=
    assign branch_info_request_o      = mispredicted_r;
    assign branch_info_is_taken_o     = (pipe1_branch_e1_w & branch_exec1_is_taken_i)     | (pipe0_branch_e1_w & branch_exec0_is_taken_i);
    assign branch_info_is_not_taken_o = (pipe1_branch_e1_w & branch_exec1_is_not_taken_i) | (pipe0_branch_e1_w & branch_exec0_is_not_taken_i);
    assign branch_info_is_call_o      = (pipe1_branch_e1_w & branch_exec1_is_call_i)      | (pipe0_branch_e1_w & branch_exec0_is_call_i);
    assign branch_info_is_ret_o       = (pipe1_branch_e1_w & branch_exec1_is_ret_i)       | (pipe0_branch_e1_w & branch_exec0_is_ret_i);
    assign branch_info_is_jmp_o       = (pipe1_branch_e1_w & branch_exec1_is_jmp_i)       | (pipe0_branch_e1_w & branch_exec0_is_jmp_i);
    assign branch_info_source_o       = (pipe1_branch_e1_w & branch_exec1_request_i)      ? branch_exec1_source_i : branch_exec0_source_i;
    assign branch_info_pc_o           = (pipe1_branch_e1_w & branch_exec1_request_i)      ? branch_exec1_pc_i     : branch_exec0_pc_i;
    ```
- Hazard Detection
    * Data Hazard: Load/Store, if load store operation not finished when reaching e2 -> Stall
    ```verilog=
    wire   load_store_e2_w = ctrl_e2_q[`PCINFO_LOAD] | ctrl_e2_q[`PCINFO_STORE];
    assign load_e2_o       = ctrl_e2_q[`PCINFO_LOAD];
    assign mul_e2_o        = ctrl_e2_q[`PCINFO_MUL];
    assign rd_e2_o         = {5{(valid_e2_w && ctrl_e2_q[`PCINFO_RD_VALID] && ~stall_o)}} & opcode_e2_q[`RD_IDX_R];
    assign result_e2_o     = result_e2_r;
    assign stall_o         = (ctrl_e1_q[`PCINFO_DIV] && ~div_complete_i) || ((ctrl_e2_q[`PCINFO_LOAD] | ctrl_e2_q[`PCINFO_STORE]) & ~mem_complete_i);
    ```
  - Control Hazard
    - Prediction wrong: Flush instructions and results
    ```verilog=
    begin
        valid_e1_q      <= 1'b0;
        ctrl_e1_q       <= `PCINFO_W'b0;
        pc_e1_q         <= 32'b0;
        npc_e1_q        <= 32'b0;
        opcode_e1_q     <= 32'b0;
        operand_ra_e1_q <= 32'b0;
        operand_rb_e1_q <= 32'b0;
        exception_e1_q  <= `EXCEPTION_W'b0;
    end
    ```
- biRISC-V supports fully-bypassing, and the code is shown as following: 
    :::spoiler Fully-Bypassing
    ```verilog=
    always @ *
    begin
        // NOTE: Newest version of operand takes priority
        issue_a_ra_value_r = issue_a_ra_value_w;
        issue_a_rb_value_r = issue_a_rb_value_w;

        // Bypass - WB
        if (pipe0_rd_wb_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = pipe0_result_wb_w;
        if (pipe0_rd_wb_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = pipe0_result_wb_w;

        if (pipe1_rd_wb_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = pipe1_result_wb_w;
        if (pipe1_rd_wb_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = pipe1_result_wb_w;

        // Bypass - E2
        if (pipe0_rd_e2_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = pipe0_result_e2_w;
        if (pipe0_rd_e2_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = pipe0_result_e2_w;

        if (pipe1_rd_e2_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = pipe1_result_e2_w;
        if (pipe1_rd_e2_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = pipe1_result_e2_w;

        // Bypass - E1
        if (pipe0_rd_e1_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = writeback_exec0_value_i;
        if (pipe0_rd_e1_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = writeback_exec0_value_i;

        if (pipe1_rd_e1_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = writeback_exec1_value_i;
        if (pipe1_rd_e1_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = writeback_exec1_value_i;

        // Reg 0 source
        if (issue_a_ra_idx_w == 5'b0)
            issue_a_ra_value_r = 32'b0;
        if (issue_a_rb_idx_w == 5'b0)
            issue_a_rb_value_r = 32'b0;
    end
    ```
    :::
* Control Unit send issue to hardware by the opcode of instruction
    :::spoiler Controller issue
    ```verilog=
    //-------------------------------------------------------------
    // Load store unit
    //-------------------------------------------------------------
    assign lsu_opcode_opcode_o      = pipe1_mux_lsu_r ? opcode1_opcode_o     : opcode0_opcode_o;
    assign lsu_opcode_pc_o          = pipe1_mux_lsu_r ? opcode1_pc_o         : opcode0_pc_o;
    assign lsu_opcode_rd_idx_o      = pipe1_mux_lsu_r ? opcode1_rd_idx_o     : opcode0_rd_idx_o;
    assign lsu_opcode_ra_idx_o      = pipe1_mux_lsu_r ? opcode1_ra_idx_o     : opcode0_ra_idx_o;
    assign lsu_opcode_rb_idx_o      = pipe1_mux_lsu_r ? opcode1_rb_idx_o     : opcode0_rb_idx_o;
    assign lsu_opcode_ra_operand_o  = pipe1_mux_lsu_r ? opcode1_ra_operand_o : opcode0_ra_operand_o;
    assign lsu_opcode_rb_operand_o  = pipe1_mux_lsu_r ? opcode1_rb_operand_o : opcode0_rb_operand_o;
    assign lsu_opcode_invalid_o     = 1'b0;

    //-------------------------------------------------------------
    // Multiply
    //-------------------------------------------------------------
    assign mul_opcode_opcode_o      = pipe1_mux_mul_r ? opcode1_opcode_o     : opcode0_opcode_o;
    assign mul_opcode_pc_o          = pipe1_mux_mul_r ? opcode1_pc_o         : opcode0_pc_o;
    assign mul_opcode_rd_idx_o      = pipe1_mux_mul_r ? opcode1_rd_idx_o     : opcode0_rd_idx_o;
    assign mul_opcode_ra_idx_o      = pipe1_mux_mul_r ? opcode1_ra_idx_o     : opcode0_ra_idx_o;
    assign mul_opcode_rb_idx_o      = pipe1_mux_mul_r ? opcode1_rb_idx_o     : opcode0_rb_idx_o;
    assign mul_opcode_ra_operand_o  = pipe1_mux_mul_r ? opcode1_ra_operand_o : opcode0_ra_operand_o;
    assign mul_opcode_rb_operand_o  = pipe1_mux_mul_r ? opcode1_rb_operand_o : opcode0_rb_operand_o;
    assign mul_opcode_invalid_o     = 1'b0;

    //-------------------------------------------------------------
    // CSR unit
    //-------------------------------------------------------------
    assign csr_opcode_valid_o       = opcode_a_issue_r & ~take_interrupt_i;
    assign csr_opcode_opcode_o      = opcode0_opcode_o;
    assign csr_opcode_pc_o          = opcode0_pc_o;
    assign csr_opcode_rd_idx_o      = opcode0_rd_idx_o;
    assign csr_opcode_ra_idx_o      = opcode0_ra_idx_o;
    assign csr_opcode_rb_idx_o      = opcode0_rb_idx_o;
    assign csr_opcode_ra_operand_o  = opcode0_ra_operand_o;
    assign csr_opcode_rb_operand_o  = opcode0_rb_operand_o;
    assign csr_opcode_invalid_o     = opcode_a_issue_r && issue_a_invalid_w;
    ```
    :::

    
### ALU(E1) Stage
There are some operations in this stage, `ALU`, `LSU`, `CSU` and `MUL`
* `ALU`: Arithemtic  Operation
* `LSU`: Load/Store Operation
* `CSU`: Deal with Interrupt and Exception
* `MUL`: Pipelining Multiplier

#### ALU
- Control
  In `biriscv_defs.v` we can see some definition for ALU operation:
  ```verilog=
    `define ALU_NONE                                4'b0000
    `define ALU_SHIFTL                              4'b0001
    `define ALU_SHIFTR                              4'b0010
    `define ALU_SHIFTR_ARITH                        4'b0011
    `define ALU_ADD                                 4'b0100
    `define ALU_SUB                                 4'b0110
    `define ALU_AND                                 4'b0111
    `define ALU_OR                                  4'b1000
    `define ALU_XOR                                 4'b1001
    `define ALU_LESS_THAN                           4'b1010
    `define ALU_LESS_THAN_SIGNED                    4'b1011
  ```
  
  :::spoiler I/O Ports in ALU. 
  ```verilog=
    module biriscv_alu
    (
        // Inputs
        input  [  3:0]  alu_op_i         // opcode in the instruction
        ,input  [ 31:0]  alu_a_i         // register 1
        ,input  [ 31:0]  alu_b_i         // register 2

        // Outputs
        ,output [ 31:0]  alu_p_o         // destinated register
    );
  ```
  :::
  From I/O, We can plot a simple graph. 
  ![](https://i.imgur.com/B56YUmj.jpg)
  
  There are some differenct operations, it can decide the normal arithmetic operation like `add` and `sub`, logical operation like `and` and `or`, Comparison like `less than` and `less than signed` and the most important operation, bitwise operation, `shift left`, `shift right`, and `shift right with signed bit`. In the source code, there is some variable to decide the bit number to do the shift operation
  ```verilog=
    shift_right_fill_r = 16'b0;
    shift_right_1_r = 32'b0;
    shift_right_2_r = 32'b0;
    shift_right_4_r = 32'b0;
    shift_right_8_r = 32'b0;

    shift_left_1_r = 32'b0;
    shift_left_2_r = 32'b0;
    shift_left_4_r = 32'b0;
    shift_left_8_r = 32'b0;
  ```
 
- Datapath
  The following source code show the result singal of ALU. `result_r` is a register which stored the result of instruction, and `alu_p_o` is the result signal of ALU.
  ```verilog=
    assign alu_p_o    = result_r;
  ```
#### LSU
Load Store Unit used to load and store...
:::spoiler I/O Ports
```verilog=
(
    // Inputs
     input           clk_i                // Clock
    ,input           rst_i                // Reset
    ,input           opcode_valid_i       
    ,input  [ 31:0]  opcode_opcode_i      // input instruction, used to know which instruction is
    ,input  [ 31:0]  opcode_pc_i
    ,input           opcode_invalid_i
    ,input  [  4:0]  opcode_rd_idx_i      
    ,input  [  4:0]  opcode_ra_idx_i
    ,input  [  4:0]  opcode_rb_idx_i
    ,input  [ 31:0]  opcode_ra_operand_i
    ,input  [ 31:0]  opcode_rb_operand_i
    ,input  [ 31:0]  mem_data_rd_i
    ,input           mem_accept_i
    ,input           mem_ack_i
    ,input           mem_error_i
    ,input  [ 10:0]  mem_resp_tag_i
    ,input           mem_load_fault_i     // memory fault
    ,input           mem_store_fault_i

    // Outputs
    ,output [ 31:0]  mem_addr_o
    ,output [ 31:0]  mem_data_wr_o
    ,output          mem_rd_o
    ,output [  3:0]  mem_wr_o
    ,output          mem_cacheable_o      // cache hit?
    ,output [ 10:0]  mem_req_tag_o        // tag bit
    ,output          mem_invalidate_o     // valid bit
    ,output          mem_writeback_o      // writeback 
    ,output          mem_flush_o           
    ,output          writeback_valid_o
    ,output [ 31:0]  writeback_value_o
    ,output [  5:0]  writeback_exception_o
    ,output          stall_o
);
```
:::

:::info
CSR and MUL implement from the ALU stage(E1) to writeback stage(WB)
:::

#### CSR
CSR(Control and Status Register) is used to deal with Exception, Interrupt and Storage Protection. There are two mode in CSR, machine mode and supervisor mode. In the standard user-level base ISA, only a handful of read-only counter CSRs are accessible.

- Next State if the 
    - When the Interrupt enhanced in machine mode, CSR will save interrupt state, disabled it, enter supervisor mode and raise the priviledge to machine level.
        :::spoiler source code
        ```verilog=
          // Interrupts
            if ((exception_i & `EXCEPTION_TYPE_MASK) == `EXCEPTION_INTERRUPT)
            begin
                // Machine mode interrupts
                if (irq_priv_q == `PRIV_MACHINE)
                begin
                    // Save interrupt / supervisor state
                    csr_sr_r[`SR_MPIE_R] = csr_sr_r[`SR_MIE_R];
                    csr_sr_r[`SR_MPP_R]  = csr_mpriv_q;

                    // Disable interrupts and enter supervisor mode
                    csr_sr_r[`SR_MIE_R]  = 1'b0;

                    // Raise priviledge to machine level
                    csr_mpriv_r          = `PRIV_MACHINE;

                    // Record interrupt source PC
                    csr_mepc_r           = exception_pc_i;
                    csr_mtval_r          = 32'b0;

                    // Piority encoded interrupt cause
                    if (interrupt_o[`IRQ_M_SOFT])
                        csr_mcause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_M_SOFT;
                    else if (interrupt_o[`IRQ_M_TIMER])
                        csr_mcause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_M_TIMER;
                    else if (interrupt_o[`IRQ_M_EXT])
                        csr_mcause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_M_EXT;
                end
        ```
        :::

    - When the Interrupt enhanced in supervisor mode, CSR will save interrupt state, disabled it, enter supervisor mode and raise the priviledge to super level.
        :::spoiler source code
        ```verilog=
        begin
                // Save interrupt / supervisor state
                csr_sr_r[`SR_SPIE_R] = csr_sr_r[`SR_SIE_R];
                csr_sr_r[`SR_SPP_R]  = (csr_mpriv_q == `PRIV_SUPER);

                // Disable interrupts and enter supervisor mode
                csr_sr_r[`SR_SIE_R]  = 1'b0;

                // Raise priviledge to machine level
                csr_mpriv_r  = `PRIV_SUPER;

                // Record fault source PC
                csr_sepc_r   = exception_pc_i;
                csr_stval_r  = 32'b0;

                // Piority encoded interrupt cause
                if (interrupt_o[`IRQ_S_SOFT])
                    csr_scause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_S_SOFT;
                else if (interrupt_o[`IRQ_S_TIMER])
                    csr_scause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_S_TIMER;
                else if (interrupt_o[`IRQ_S_EXT])
                    csr_scause_r = `MCAUSE_INTERRUPT + 32'd`IRQ_S_EXT;
            end
        ```
        :::

#### MUL
Here we will introduce the multiplier implemented in this project, multiplier is in `biriscv_multiplier.v`
- I/O ports
    ```verilog=
    module biriscv_multiplier
    (
        // Inputs
         input           clk_i                // clock
        ,input           rst_i                // reset
        ,input           opcode_valid_i       // boolean for valid bie
        ,input  [ 31:0]  opcode_opcode_i      // type of multiple operation, MUL, MULH, MULHSU...
        ,input  [ 31:0]  opcode_pc_i
        ,input           opcode_invalid_i     // redundant variable here, not used
        ,input  [  4:0]  opcode_rd_idx_i      
        ,input  [  4:0]  opcode_ra_idx_i
        ,input  [  4:0]  opcode_rb_idx_i
        ,input  [ 31:0]  opcode_ra_operand_i
        ,input  [ 31:0]  opcode_rb_operand_i
        ,input           hold_i

        // Outputs
        ,output [ 31:0]  writeback_value_o    // write back value
    );
    ```
- Datapath
```verilog=
assign mult_result_w = {{ 32 {operand_a_e1_q[32]}}, operand_a_e1_q}*{{ 32 {operand_b_e1_q[32]}}, operand_b_e1_q};
```
- This multiplier is using  a `*` and it implemented by verilog EDA, and EDA will make a pipelined multiplier simultaneously.
![](https://i.imgur.com/qjTjNPt.png)


### Mem(E2) Stage
- DCache(Data Cache): similar to icache
- MMU(Memory Manage Unit)
  - Memory Management Unit is used to translate virtual memory address to physical memory address, which we learned in CS61C virtual memory. However, in `biriscv`, there are supervisor mode and user mode, so before access the TLB, we need to know it is user page or supervisor page. Supervisor cannot access user page
  - Here is TLB(Translation Lookaside Buffer) which is used to convert virtual address to physical address

![](https://i.imgur.com/gKt1ay8.png)

### Writeback Stage
There is no fixed Write Back stage, because when the controller issue the instruction to different hardware corresponding their operation, there are different time to finish it. For example, `ALU` only take one stage `e1`, then the operation is done. However, `MUL` take three stage from `e1` to `W`. When `ALU` is finished, it will write back to memory, and so does `MUL`, the shemantic figure is shown as below:
![](https://i.imgur.com/dPMzzuY.png)

The figure shown above is a x86-64 architecture not riscv, but the concept is similar. The figure above is describe the operation of int and float take different time, and when the operation is finished, it will concatenate write back Stage. Thus Write Back Stage is not fixed, maybe it appears in stage 6 or stage 7.


## Requirement 2, Verilator Analyze
In GTKwave, we mainly use the following siganl to analyze.
:::info
- next_pc_f_o: The next address of instruction that will be fetched by processor. (From biriscv_npc.v)
- pc_f_q: The address of instruction is fetched from icache (From biriscv_fetch.v) 
- pc_d_q: The address of instruction will be decoded by decoder (From biriscv_fetch.v)
- fetch_out0_pc_o: The address of instruction is decoded by decoder0 (From biriscv_decode.v), same as fetch0_pc_i in biriscv_issue.v
- fetch_out1_pc_o: The address of instruction is decoded by decoder1 (From biriscv_decode.v)
- pc_x_q: The address of instruction in Issue stage (From biriscv_issue.v)
`pipe0`
- pc_e1_q: The address of instruction in E1 stage (From biriscv_pipe_ctrl.v)
- pc_e2_q: The address of instruction in E2 stage (From biriscv_pipe_ctrl.v)
- pc_wb_q: The address of instruction in WB stage (From biriscv_pipe_ctrl.v)
`pipe1`
- pc_e1_q: The address of instruction in E1 stage (From biriscv_pipe_ctrl.v)
- pc_e2_q: The address of instruction in E2 stage (From biriscv_pipe_ctrl.v)
- pc_wb_q: The address of instruction in WB stage (From biriscv_pipe_ctrl.v)
:::
![](https://i.imgur.com/Z3069Lp.png)

### RV32I
- I-Format
  We observe the following code from test.elf, especially `lw	a5,-448(s0)`
    ```verilog=
    800011d0:	e4042783          	lw	a5,-448(s0)
    800011d4:	00148493          	addi	s1,s1,1
    800011d8:	000780e7          	jalr	a5
    800011dc:	0004c503          	lbu	a0,0(s1)
    800011e0:	fe0518e3          	bnez	a0,800011d0 <puts+0x38>
    ```
    The following picture is the result, and `lw	a5,-448(s0)` is executed in pipe0.
    1. Red box is the instruction in pipeline
    2. Yellow Box is the result of the instruction
    
    ![](https://i.imgur.com/fZGHhkM.png)

- R-Format
  We observe the following code from test.elf, especially `add a4, a4, a3`
  ```asm=
    800006a0:	df042683          	lw	a3,-528(s0)
    800006a4:	000037b7          	lui	a5,0x3
    800006a8:	69c78793          	addi	a5,a5,1692 # 369c <boot_vector-0x7fffc964>
    800006ac:	00169713          	slli	a4,a3,0x1
    800006b0:	00d70733          	add	a4,a4,a3
    800006b4:	dee42823          	sw	a4,-528(s0)
    800006b8:	df042703          	lw	a4,-528(s0)
    800006bc:	02f70263          	beq	a4,a5,800006e0 <main+0xbc>         	lw	a4,-528(s0)
  ```
  The following picture is the result, and `add a4, a4, a3` is executed in pipe0.
  1. The red boxes represent the flow of `add a4, a4, a3`, and we can observe the instruction from `PC stage` to `WB stage`
  2. The yellow boxes represent the stall in order to wait the next instruction into `issue stage`.
  3. We find out there is a data hazard (blue box), at `slli a4, a3, 0x1` and `add a4, a4, a3`. From the following source code in `biriscv_issue.v`, we can know how the biRISC-V deal with the data hazard.
     ```verilog=
        // Bypass - E1
        if (pipe0_rd_e1_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = writeback_exec0_value_i;
        if (pipe0_rd_e1_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = writeback_exec0_value_i;
     ```
     Now, we know that the `pipe0_rd_e1_w` is the `a4` in `slli a4, a3, 0x1` and `issue_a_ra_idx_w` is the `a4` in `add a4, a4, a3`. So, the result of E1 stage will bypass to Issue stage.
     
     ![](https://i.imgur.com/bfu0ayK.png)
  
![](https://i.imgur.com/pk1P5yW.png)

- SB-Format
  We observe the following code from test.elf, especially `beq a4,a5,80000688`
  ```asm=
    80000658:	df042703          	lw	a4,-528(s0) # 80001df0 <_end+0xffffefa0>
    8000065c:	000017b7          	lui	a5,0x1
    80000660:	23478793          	addi	a5,a5,564 # 1234 <boot_vector-0x7fffedcc>
    80000664:	02f70263          	beq	a4,a5,80000688 <main+0x64>
    80000668:	80002637          	lui	a2,0x80002
    8000066c:	800025b7          	lui	a1,0x80002
    80000670:	80002537          	lui	a0,0x80002
  ```
  The following picture is the result, and `beq	a4,a5,80000688` is executed in pipe0.
  :::info
  - pipe0_branch_e1_w: Whether the instruction in E1 stage is branch instruction.
  - branch_exec0_is_taken_i: the branch instruction is taken.
  :::
  1. We can know the branch prediction is the next instruction.
  2. When branch instruction in the E1 stage, biRISC-V will detect and determine that the branch is taken or not.
  3. There is a data hazard, at `addi a5, a5, 564` and `beq	a4, a5, 80000688`. From last example, we can know the result of `addi` will bypass to `beq`.
  4. There are total 4 stalls, 3 branch penalty + 1 wait instruction to issue (But branch is taken, there is no any change).
  
![](https://i.imgur.com/zPnDxIb.png)

### M standard extension
- Multiplication Operations
  We observe the following code from test.elf, especially `mul a4,a4,a5`
  :::info
  mul rd, rs1, rs2 
  &rarr; MUL performs an XLEN-bit(rs1) × XLEN-bit(rs2) multiplication and places the lower XLEN bits in the destination register(rd). In RV32IM, XLEN-bit means 32-bit.
  
  - mult_inst_w: Whether the multiple instruction in the Issue stage.
  - pipe0_mul_e1_w: Whether multiple instruction in the E1 stage. (In pipe0)
  - mulhi_sel_e1_q: Choose the higher XLEN bit or lower XLEN bit.
  - writeback_value_o: the result is bypassed from E2 or E3(WB) stage.
  :::
  ```asm=
    800007c4:	def42823          	sw	a5,-528(s0)
    800007c8:	df042703          	lw	a4,-528(s0)
    800007cc:	009387b7          	lui	a5,0x938
    800007d0:	83378793          	addi	a5,a5,-1997 # 937833 <boot_vector-0x7f6c87cd>
    800007d4:	02f70733          	mul	a4,a4,a5
    800007d8:	e68fd7b7          	lui	a5,0xe68fd
    800007dc:	b2b78793          	addi	a5,a5,-1237 # e68fcb2b <_end+0x668f9cdb>
  ```
  The following picture is the result, and `mul a4,a4,a5` is executed in pipe0.
  1. Because mul chooses the lower XLEN of result, from the source code, we can know how the multipler work.
     ```verilog=
        mulhi_sel_e1_q <= ~((opcode_opcode_i & `INST_MUL_MASK) == `INST_MUL);
        result_r = mulhi_sel_e1_q ? mult_result_w[63:32] : mult_result_w[31:0];
     ```
     From source code, mulhi_sel_e1_q is equal to 0 and result_r is equal to mult_result_w[31:0] (choose lower XLEN bit).
  2. There is a data hazard between `addi a5,a5,-1997` and `mul	a4,a4,a5`, and we check the `issue_a_rb_idx_w` and `pipe0_rd_e1_w`.
     ```verilog=
        // Bypass - E1
        if (pipe0_rd_e1_w == issue_a_ra_idx_w)
            issue_a_ra_value_r = writeback_exec0_value_i;
        if (pipe0_rd_e1_w == issue_a_rb_idx_w)
            issue_a_rb_value_r = writeback_exec0_value_i;     
     ```
     The result will bypass from E1 stage to Issue stage.
  3. From picture, we find out that the result writeback at E2 stage. 
  4. Follow the source code, `MULT_STAGES` control the value writeback at E2 stage or WB stage.
     ```verilog=
         localparam MULT_STAGES = 2; // 2 or 3
         writeback_value_o  = (MULT_STAGES == 3) ? result_e3_q : result_e2_q;
     ```
![](https://i.imgur.com/1LdP038.png)

- Division Operations
  We observe the following code from test.elf, especially `div  a4,a4,a5`
  ```asm=
    8000081c:	69c78793          	addi	a5,a5,1692 # 369c <boot_vector-0x7fffc964>
    80000820:	def42823          	sw	a5,-528(s0)
    80000824:	df042703          	lw	a4,-528(s0)
    80000828:	00500793          	li	a5,5
    8000082c:	02f74733          	div	a4,a4,a5
    80000830:	000017b7          	lui	a5,0x1
    80000834:	aec78793          	addi	a5,a5,-1300 # aec <boot_vector-0x7ffff514>
    80000838:	dee42823          	sw	a4,-528(s0)
  ```
  The following picture is the result, and mul a4,a4,a5 is executed in pipe0.
  1. There are 34 cycles for executing div instruction.
     ```verilog=
         // Division operations take 2 - 34 cycles and stall
         // the pipeline (complete out-of-pipe) until completed.
        always @ (posedge clk_i or posedge rst_i)
        if (rst_i)
            div_pending_q <= 1'b0;
        else if (pipe0_squash_e1_e2_w || pipe1_squash_e1_e2_w)
            div_pending_q <= 1'b0;
        else if (div_opcode_valid_o && issue_a_div_w)
            div_pending_q <= 1'b1;
        else if (writeback_div_valid_i)
            div_pending_q <= 1'b0;
     ```
     From the source code, we can know that the divider will take 2 ~ 34 cycles and stall.
  2. The result is complete in divider and writeback into E1 stage.
  
![](https://i.imgur.com/h0pgbT8.png)


### Control and Status Registers(Zicsr)
- Example 1: csrw (`csrw csr, rs1`, is encoded as `csrrw x0, csr, rs1`)
  :::info
  The CSRRW (Atomic Read/Write CSR) instruction atomically swaps values in the CSRs and integer registers. CSRRW reads the old value of the CSR, zero-extends the value to XLEN bits, then writes it to integer register rd. The initial value in rs1 is written to the CSR. If rd=x0, then the instruction shall not read the CSR and shall not cause any of the side effects that might occur on a CSR read.
  
  - issue_csr_i: Detect the csr instruction is in Issue stage or not.
  - issue_a_rb_idx_w: The rs2 of the instruction in Issue stage.
  - pipe0_rd_e1_w: The rd of the instruction in E1 stage.
  - issue_a_ra_value_r: The register gets the bypassing value and represents new rs1.
  - writeback_exec0_value_i: The value bypassing from E1 stage to Issue stage.
  - csr_e1_w: Detect the csr instruction is in E1 stage or not.
  - csr_pending: Stall the pipeline (avoid any complications).
  - csr_result_e1_wdata_o: The result of csr instruction in E1 stage.
  - x5_t0_w: The value of t0 register.
  - csr_mtvec_q: The value of mtvec register in csr register files.
  :::
  
  We observe the following code from test.elf, especially `csrw  a4,a4,a5`
  ```asm=
    80000264:	e4010113          	addi	sp,sp,-448 # 80002e40 <_end+0xfffffff0>
    80000268:	800002b7          	lui	t0,0x80000
    8000026c:	12428293          	addi	t0,t0,292 # 80000124 <_end+0xffffd2d4>
    80000270:	30529073          	csrw	mtvec,t0
    80000274:	800022b7          	lui	t0,0x80002
    80000278:	df828293          	addi	t0,t0,-520 # 80001df8 <_end+0xffffefa8>
  ```
  The following picture is the result, and `csrw mtvec,t0` is executed in pipe0.
  1. There are 3 cycles for executing csr instruction.
     ```verilog=
        // CSR operations are infrequent - avoid any complications of pipelining them.
        // These only take a 2-3 cycles anyway and may result in a pipe flush (e.g. ecall, ebreak..).
        always @ (posedge clk_i or posedge rst_i)
        if (rst_i)
            csr_pending_q <= 1'b0;
        else if (pipe0_squash_e1_e2_w || pipe1_squash_e1_e2_w)
            csr_pending_q <= 1'b0;
        else if (csr_opcode_valid_o && issue_a_csr_w)
            csr_pending_q <= 1'b1;
        else if (pipe0_csr_wb_w)
            csr_pending_q <= 1'b0;
     ```
     From the source code, we can know that the pipeline will take 2 ~ 3 cycles and stall.
  
  2. In the picture, we can see that the value of t0 register copy to the mtvec register.

![](https://i.imgur.com/O5PzNyW.png)

## Requirement 3, GTKWave for Dual Issue Analyze
### Dual Issue
In verilog code, controller can issue all operations to Pipeline 0(`issue a`). However, it can only issue `lsu`, `branch`, `alu` and `mul` to Pipeline 1(`issue b`). In this case, controller issue all but `mul`, the source code of dual issue is shown as following:
:::info
**Parameters in `issue.v`**
* `fetch0_instr_exec_i=1`: when Pipeline 0 fetch `ANDI`, `ADDI`, `SLTI`, `SLTIU`, `ORI`, `SLLI`, `ADD`, `AUIPC`...
* `fetch0_instr_lsu_i=1`: when Pipeline 0 fetch `LB`, `LH`, `LW`, `LBH`, `LHU`, `LWU`, `SB`, `SH` or `SW`
* `fetch0_instr_branch_i=1`: when Pipeline 0 fetch `JAL`, `JALR`, `BEQ`, `BNE`, `BLT`, `BGE`, `BLTU` or `BGEU`
* `fetch0_instr_mul_i=1`: when Pipeline 0 fetch `MUL`, `MULH`, `MULHSU` or `MULHU`
* `fetch0_instr_div_i=1`: when Pipeline 0 fetch `DIV`, `DIVU`, `REM` or `REMU`
* `fetch0_instr_csr_i=1`: when Pipeline 0 fetch `ECALL`, `EBREAK`, `ERET`, `CSRRW`, `CSRRS`, `CSRRC`, `CSRRWI`...
* `pipe1_ok_w`: instruction is `lsu`, `branch`, `alu` or `mul`
* `take_interrupt_i`: boolean value for interrupt enhanced
* `enable_dual_issue_w`: this value is determined by `SUPPORT_DUAL_ISSUE`
:::

The following source code shows about what condition can occur the dual issue.
```verilog=
// Is this combination of instructions possible to execute concurrently.
// This excludes result dependencies which may also block secondary execution.
wire dual_issue_ok_w =   enable_dual_issue_w &&  // Second pipe switched on
                         pipe1_ok_w &&           // Instruction 2 is possible on second exec unit
                        (((issue_a_exec_w | issue_a_lsu_w | issue_a_mul_w) && issue_b_exec_w)   ||
                         ((issue_a_exec_w | issue_a_lsu_w | issue_a_mul_w) && issue_b_branch_w) ||
                         ((issue_a_exec_w | issue_a_mul_w) && issue_b_lsu_w)                    ||
                         ((issue_a_exec_w | issue_a_lsu_w) && issue_b_mul_w)
                         ) && ~take_interrupt_i;
```
`dual_issue_ok_w` is determined by `enable_dual_issue_w`, `pipe1_ok_w`, `take_interrupt_i`, and others.

From this code, we can summary 3 conditions that can occur the dual issue.
1. `exec`, `lsu` or `mul` in decoder0 and `exec` or `branch` in decoder1.
2. `exec` or `mul` in decoder0 and `lsu` in decoder1
3. `exec` or `lsu` in decoder0 and `mul` in decoder1

So now, let's discuss the parameters of `dual_issue_w`, the parameters in the code above.
When `opcode_b_issue_r`, `opcode_b_accept_r` and `~take_interrupt_i` are all equal to 1, the result of `dual_issue_w` is 1, and the dual issue will start with next cycle.

```verilog=
    assign dual_issue_w  = opcode_b_issue_r & opcode_b_accept_r & ~take_interrupt_i;
```

### Example
Here are an example for dual issue:
```verilog=
    80000284 <bss_clear>:
    80000284:	028000ef          	jal	ra,800002ac <init>
    80000288:	80000537          	lui	a0,0x80000
    8000028c:	02050513          	addi	a0,a0,32 # 80000020 <_end+0xffffd1d0>
    80000290:	00052503          	lw	a0,0(a0)
    80000294:	800005b7          	lui	a1,0x80000
    80000298:	02458593          	addi	a1,a1,36 # 80000024 <_end+0xffffd1d4>
    8000029c:	388000ef          	jal	ra,80000624 <main>
```
* No Dual Issue

![](https://i.imgur.com/ufpcOFI.png)
  1. check whether this processor is support dual issue. This processor is truely support.
  2. we can find the `issue_a_exec_w` and `issue_b_exec_w` are equal to 1. Therefore, `dual_issue_ok_w` is equal to 1, too.
  3. `opcode_b_issue_r` and `opcode_b_accept` are not equal to 1, so `dual_issue_w` is equal to 0.
  4. this case doesn't occur the dual issue.

* Dual Issue: `80000294 lui a0, 0x80000` and `8000029c jal ra, 80000624<main>` 

![](https://i.imgur.com/9HFthFk.png)
  1. check whether this processor is support dual issue. This processor is truely support.
  2. In Red and Yellow box, we can find the `issue_a_lsu_w` and `issue_b_exec_w` are equal to 1. Therefore, `dual_issue_ok_w` is equal to 1, too.
  3. In Blue and Purple box, we can find the `issue_a_exec_w` and `issue_b_branch_w` are equal to 1. Therefore, `dual_issue_ok_w` is equal to 1, too.
  4. `opcode_b_issue_r` and `opcode_b_accept` are equal to 1 in both case, so `dual_issue_w` is equal to 1.
  5. this case will occur the dual issue.

## Reference
* [The RISC-V Instruction Set Manual](https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf)
* [Design of a dual-issue RISC-V processor](https://iopscience.iop.org/article/10.1088/1742-6596/1693/1/012192/pdf)
* [Verilog Note](https://hackmd.io/08elmgP4SUS1VQP2jb7Jmg)
* [biRISC-V github](https://github.com/ultraembedded/biriscv)
* [現代處理器設計：原理和關鍵特徵](http://hackfoldr.org/cpu/https%253A%252F%252Fhackmd.io%252Fs%252FHk2CscGcl)

## Appendix
- Branch predictor
  - BHT: Branch History Table**
    Branch History Table is a table to record the branch information, and which is used to determine whether this branch instruction be taken. Here is one bit to record the branch prediction. However, one bit branch prediction may cause two misprediction. Thus, there are at least two bits for branch prediction.
      ![](https://i.imgur.com/dtFKqdp.png)
    Reference: https://users.ece.cmu.edu/~jhoe/course/ece447/S21handouts/L10.pdf
    Now, return to `biriscv`, I think there is only one bit to do branch prediction.
    ```verilog=
        ,input           branch_is_taken_i   
        ,input           branch_is_not_taken_i 
    ```
  
  - BTB: Branch Target Buffer**
    ![](https://i.imgur.com/XUpXKtl.png)
    ![](https://i.imgur.com/p1KilrI.png)

    Reference: https://users.ece.cmu.edu/~jhoe/course/ece447/S21handouts/L10.pdf
  
    BTB Architecture is shown above. After branch instruction is implemented, the address of instruction and the jump address. 
  
    How to use BTB?
  
    We will compare the Program Counter and the address of branch instrution in the BTB, if we found that is in the BTB, we will use the address which is predicted by BTB. If none, just go to PC+4.
  
  - RAS: Return Address Stack**
    A stack record the return address.
  
  - GSHARE**
    ![](https://i.imgur.com/o2rBGEp.png)
  
    GShare is a dynamic brnach predictor, which include BTB and BHT. When a instruction come, GShare will query BTB, if hits, then query BHT to predict whether a branch will occur.
    > Static Branch Prediction: Predict without history information 
    > Dynamic Branch Prediction: Predict with history information
  
    Take N-bit from branch instruction and M-bit from Branch History Shift Register(BHSR) to search the table shown as above, then predict next PC

- x86-64 Decode Stage
    Here we will introduce the decode stage in biriscv, and there are two decoder This figure is not the arichtecture of biriscv, this is x86-64 architecture. After I see the verilog code, I find that there are quite similar, and it make me understand the decode stage in multithreading architecture 
![](https://i.imgur.com/hWap3qJ.png)
