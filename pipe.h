#ifndef _PIPE_H_
#define _PIPE_H_

#include "shell.h"
#include "stdbool.h"
#include <limits.h>

#define MIPS_REGS 32

typedef struct CPU_State_Struct {
  uint32_t PC;		/* program counter */
  int32_t REGS[MIPS_REGS]; /* register file. */
  int FLAG_N;        /* negative flag or sign flag*/
  int FLAG_Z;        /* zero flag */
  int FLAG_V;        /* overflow flag */
  int FLAG_C;        /* carry flag */
} CPU_State;

int RUN_BIT;

/* global variable -- pipeline state */
extern CPU_State CURRENT_STATE;

/* called during simulator startup */
void pipe_init();

/* this function calls the others */
void pipe_cycle();

//Register in between the instruction fetch and decode stages.
typedef struct Pipe_Reg_IFtoDE {   
    uint32_t PC;
    uint32_t Instruction;

    //Stall Status
    bool IFFlush;

    //Stall enable
    bool IFtoDE_Write;

} Pipe_Reg_IFtoDE;


//Register in between the decode and execute stages.
typedef struct Pipe_Reg_DEtoEX {
    
    // Write back control values
    bool RegWrite;
    bool MemtoReg;

    // Memory control values
    bool MemRead;
    bool MemWrite;

    // Execute contol values
    bool RegDst;
    bool Unsigned;
    uint32_t ALUOperation;
    bool ALUSrc;

    // Register File data
    uint32_t Reg_1;
    uint32_t Reg_2;

    // Constant values.
    int32_t immediate;

    // Destination Register Numbers
    uint32_t rs_Num;
    uint32_t rt_Num;
    uint32_t rd_Num;

} Pipe_Reg_DEtoEX;


//Register in between the execute and memory stages.
typedef struct Pipe_Reg_EXtoMEM {
    
    // Write back control values
    bool RegWrite;
    bool MemtoReg;

    // Memory control values
    //bool Branch;
    bool MemWrite;
    bool MemRead;

    // ALU Values
    uint32_t ALU_Result;

    // Register Values
    uint32_t Reg_2;
    uint32_t Reg_Rd;

} Pipe_Reg_EXtoMEM;


//Register in between the memory and write back stages.
typedef struct Pipe_Reg_MEMtoWB {

    // Write back control values
    bool RegWrite;
    bool MemtoReg;

    uint32_t Read_Data;
    uint32_t ALU_Result;
    uint32_t Reg_Rd;

} Pipe_Reg_MEMtoWB;

/* each of these functions implements one stage of the pipeline */
void pipe_stage_fetch();
void pipe_stage_decode();
void pipe_stage_execute();
void pipe_stage_mem();
void pipe_stage_wb();

#endif
