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
    bool DataPathStop;

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
    bool DataPathStop;

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
    bool DataPathStop;

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

//The ALU operations

/***************************************************************

 Function: add

 Purpose: Perform a signed addition on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t Add(int32_t reg_1, int32_t reg_2);

/***************************************************************

 Function: addu

 Purpose: Perform a unsigned addition on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t AddU(uint32_t reg_1, uint32_t reg_2);

/***************************************************************

 Function: sub

 Purpose: Perform a signed subtraction on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t Sub(int32_t reg_1, int32_t reg_2);

/***************************************************************

 Function: subu

 Purpose: Perform a unsigned subtraction on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t SubU(uint32_t reg_1, uint32_t reg_2);

/***************************************************************

 Function: slt

 Purpose: Perform a signed set less than on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t SLT(int32_t reg_1, int32_t reg_2);

/***************************************************************

 Function: sltu

 Purpose: Perform a unsigned set less than on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t SLTU(uint32_t reg_1, uint32_t reg_2);

/***************************************************************

 Function: lu

 Purpose: Perform a load upper on the second ALU input

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t LU(uint32_t reg_2);

/***************************************************************

 Function: or

 Purpose: Perform an or on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 immd: If the immediate value is being used.

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t OR(uint32_t reg_1, uint32_t reg_2);

/***************************************************************

 Function: MEM

 Purpose: Calculate the memory address

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t MEM(uint32_t reg_1, int32_t reg_2);


//Constant values

// Defintions of opcodes for instructions that are NOT functions
#define DECODE_FUNCT	0x00
// Definitions of opcodes for instructions that ARE functions
#define DECODE_ADD	0x20
#define DECODE_ADDU 0x21
#define DECODE_SUB	0x22
#define DECODE_SUBU 0x23
#define DECODE_SLT	0x2a
#define DECODE_SLTU 0x2b
#define DECODE_LUI	0x0F
#define DECODE_ORI	0x0d
#define DECODE_ADDI 0x08
#define DECODE_ADDIU 0x09
#define DECODE_LW	0x23
#define DECODE_SW	0x2b
#define DECODE_BNE	0x05
#define DECODE_BEQ	0x04
#define DECODE_BGTZ 0x07
#define DECODE_SLTI 0x0a
#define DECODE_J 0x02

//ALU opcodes
#define EXECUTE_NO_OP 0x00
#define EXECUTE_ADD 0x01
#define EXECUTE_SUB 0x02
#define EXECUTE_SLT 0x03
#define EXECUTE_LU 0x04
#define EXECUTE_OR 0x05
#define EXECUTE_MEM 0x06

//The max value that an unsigned 32 bit int can hold
#define MAX_UNSIGNED 0xffffffff

//The max value that a signed 32 bit int can hold
#define MAX_SIGNED 0x7fffffff

//The min value that a signed 32 bit int can hold
#define MIN_SIGNED 0xffffffff


//Global Variables

//Indicates if the PC can be changed during this cycle
bool PC_Write;
//Indicates if the PC will take a branch/jump or not
bool Branch;
//The new PC from branch/jump instruction
uint32_t Branch_PC;

//Input of the Regester in between the IF and DE stages
Pipe_Reg_IFtoDE IFtoDEin;
//Regester in between the IF and DE stages
Pipe_Reg_IFtoDE IFtoDEout;

//Input of the Regester in between the DE and EX stages
Pipe_Reg_DEtoEX DEtoEXin;
//Output of the Regester in between the DE and EX stages
Pipe_Reg_DEtoEX DEtoEXout;

//Input of the Regester in between the EX and MEM stages
Pipe_Reg_EXtoMEM EXtoMEMin;
//Output of the Regester in between the EX and MEM stages
Pipe_Reg_EXtoMEM EXtoMEMout;

//Input of the Regester in between the MEM and WB stages
Pipe_Reg_MEMtoWB MEMtoWBin;
//Output of the Regester in between the MEM and WB stages
Pipe_Reg_MEMtoWB MEMtoWBout;

#endif
