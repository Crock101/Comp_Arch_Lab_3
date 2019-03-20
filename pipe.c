/**********************************************
 *
 * Program Title: Lab_3
 *
 * Program File Name: pipe.c
 *
 * Computer Architecture & Design EECE.4820-201
 *
 * Spring 2019
 *
 * Authors: Justice Graves and Colin Rockwood
 *
 * March 16th, 2019
 *
 **********************************************/

#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

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

int32_t Add(int32_t reg_1, int32_t reg_2);
uint32_t AddU(uint32_t reg_1, uint32_t reg_2);
int32_t Sub(int32_t reg_1, int32_t reg_2);
uint32_t SubU(uint32_t reg_1, uint32_t reg_2);
int32_t SLT(int32_t reg_1, int32_t reg_2);
uint32_t SLTU(uint32_t reg_1, uint32_t reg_2);
uint32_t LU(uint32_t reg_2);
uint32_t OR(uint32_t reg_1, uint32_t reg_2);
uint32_t MEM(uint32_t reg_1, int32_t reg_2);

bool PC_Write;
bool Branch;
uint32_t Branch_PC;
uint32_t Increament_PC;

//The max value that an unsigned 32 bit int can hold
#define MAX_UNSIGNED 0xffffffff

//The max value that a signed 32 bit int can hold
#define MAX_SIGNED 0x7fffffff

//The min value that a signed 32 bit int can hold
#define MIN_SIGNED 0xffffffff

Pipe_Reg_IFtoDE IFtoDE;

Pipe_Reg_DEtoEX DEtoEX;

Pipe_Reg_EXtoMEM EXtoMEM;

Pipe_Reg_MEMtoWB MEMtoWB;

void pipe_init()
{
    PC_Write = true;

    Branch = false;

    memset(&CURRENT_STATE, 0, sizeof(CPU_State));
    CURRENT_STATE.PC = 0x00400000;

    //Clear the IFtoDE reg
    IFtoDE.PC = 0;
    IFtoDE.Instruction = 0;
    IFtoDE.IFFlush = false; 
    IFtoDE.IFtoDE_Write = false;

    //Clear the DEtoEX reg
    DEtoEX.RegWrite = false;
    DEtoEX.MemtoReg = false;
    DEtoEX.MemRead = false;
    DEtoEX.MemWrite = false;
    DEtoEX.RegDst = false;
    DEtoEX.Unsigned = false;
    DEtoEX.ALUOperation = 0;
    DEtoEX.ALUSrc = false;
    DEtoEX.Reg_1 = 0;
    DEtoEX.Reg_2 = 0;
    DEtoEX.immediate = 0;
    DEtoEX.rs_Num = 0;
    DEtoEX.rt_Num = 0;
    DEtoEX.rd_Num = 0;
    DEtoEX.DataPathStop = false;

    //Clear the EXtoMEM reg
    EXtoMEM.RegWrite = false;
    EXtoMEM.MemtoReg = false;
    EXtoMEM.MemWrite = false;
    EXtoMEM.MemRead = false;
    EXtoMEM.ALU_Result = 0;
    EXtoMEM.Reg_2 = 0;
    EXtoMEM.Reg_Rd = 0;

    //Clear the MEMtoWB reg
    MEMtoWB.RegWrite = false;
    MEMtoWB.MemtoReg = false;
    MEMtoWB.Read_Data = 0;
    MEMtoWB.ALU_Result = 0;
    MEMtoWB.Reg_Rd = 0;

}

void pipe_cycle()
{
	pipe_stage_wb();
	pipe_stage_mem();
	pipe_stage_execute();
	pipe_stage_decode();
	pipe_stage_fetch();
}


void pipe_stage_wb()
{
    if (MEMtoWB.DataPathStop)
    {
        RUN_BIT = 0;
    }

    if (MEMtoWB.RegWrite)
    {
        if (MEMtoWB.MemtoReg)
        {

            CURRENT_STATE.REGS[MEMtoWB.Reg_Rd] = MEMtoWB.Read_Data;
            return;
        }

        CURRENT_STATE.REGS[MEMtoWB.Reg_Rd] = MEMtoWB.ALU_Result;

    }
}

void pipe_stage_mem()
{
    //Forwarding values to the next stage
    MEMtoWB.ALU_Result = EXtoMEM.ALU_Result;
    MEMtoWB.Reg_Rd = EXtoMEM.Reg_Rd;
    MEMtoWB.RegWrite = EXtoMEM.RegWrite;
    MEMtoWB.MemtoReg = EXtoMEM.MemtoReg;
    MEMtoWB.DataPathStop = EXtoMEM.DataPathStop;


    //Data memory segment

    //Reading data from memory?
    if (EXtoMEM.MemRead)
    {
        MEMtoWB.Read_Data = mem_read_32(EXtoMEM.ALU_Result);
    }

    //Writing data to memory?
    else if(EXtoMEM.MemWrite)
    {
        mem_write_32(EXtoMEM.ALU_Result, EXtoMEM.Reg_2);
    }
}

void pipe_stage_execute()
{
    //ALU operation

    //If an ALU op is going to occur.
    if (DEtoEX.ALUOperation != EXECUTE_NO_OP)
    {
        //Get the write back value
        uint32_t WBValue;

        if (MEMtoWB.MemtoReg)
        {
            WBValue = MEMtoWB.Read_Data;
        }
        else
        {
            WBValue = MEMtoWB.ALU_Result;
        }

        //Reg 1 EX forwarding
        if (EXtoMEM.RegWrite && (EXtoMEM.Reg_Rd != 0) && (EXtoMEM.Reg_Rd == DEtoEX.rs_Num))
        {
            DEtoEX.Reg_1 = EXtoMEM.ALU_Result;
        }
        //Reg 1 MEM forwarding
        else if (MEMtoWB.RegWrite && (MEMtoWB.Reg_Rd != 0) && (MEMtoWB.Reg_Rd == DEtoEX.rt_Num))
        {
            DEtoEX.Reg_2 = WBValue;
        }

        //Reg 2 EX forwarding
        if (EXtoMEM.RegWrite && (EXtoMEM.Reg_Rd != 0) && (EXtoMEM.Reg_Rd == DEtoEX.rt_Num))
        {
            DEtoEX.Reg_2 = EXtoMEM.ALU_Result;
        }
        //Reg 2 MEM forwarding
        else if (MEMtoWB.RegWrite && (MEMtoWB.Reg_Rd != 0) && (MEMtoWB.Reg_Rd == DEtoEX.rs_Num))
        {
            DEtoEX.Reg_1 = WBValue;
        }

        //Reg 2 value selection
        if (DEtoEX.ALUSrc)
        {
            DEtoEX.Reg_2 = DEtoEX.immediate;
        }


        if (DEtoEX.ALUOperation == EXECUTE_ADD)
        {
            if (DEtoEX.Unsigned)
            {
                EXtoMEM.ALU_Result = AddU(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
            else
            {
                EXtoMEM.ALU_Result = Add(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
        }

        else if (DEtoEX.ALUOperation == EXECUTE_SUB)
        {
            if (DEtoEX.Unsigned)
            {
                EXtoMEM.ALU_Result = SubU(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
            else
            {
                EXtoMEM.ALU_Result = Sub(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
        }

        else if (DEtoEX.ALUOperation == EXECUTE_SLT)
        {
            if (DEtoEX.Unsigned)
            {
                EXtoMEM.ALU_Result = SLTU(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
            else
            {
                EXtoMEM.ALU_Result = SLT(DEtoEX.Reg_1, DEtoEX.Reg_2);
            }
        }

        else if (DEtoEX.ALUOperation == EXECUTE_LU)
        {
            EXtoMEM.ALU_Result = LU(DEtoEX.Reg_2);
        }

        else if (DEtoEX.ALUOperation == EXECUTE_OR)
        {
            EXtoMEM.ALU_Result = OR(DEtoEX.Reg_1, DEtoEX.Reg_2);
        }

        else if (DEtoEX.ALUOperation == EXECUTE_MEM)
        {
            EXtoMEM.ALU_Result = MEM(DEtoEX.Reg_1, DEtoEX.Reg_2);
        }
    }

    //A no-op must be occuring, clear the ALU result.
    else
    {
        EXtoMEM.ALU_Result = 0;
    }

    // Forwarding control values
    EXtoMEM.RegWrite = DEtoEX.RegWrite;
    EXtoMEM.MemtoReg = DEtoEX.MemtoReg;
    EXtoMEM.MemRead = DEtoEX.MemRead;
    EXtoMEM.MemWrite = DEtoEX.MemWrite;
    EXtoMEM.DataPathStop = DEtoEX.DataPathStop;

    //Write-back regester selection
    if (DEtoEX.RegDst)
    {
        EXtoMEM.Reg_Rd = DEtoEX.rd_Num;
    }
    else
    {
        EXtoMEM.Reg_Rd = DEtoEX.rt_Num;
    }
    
}

void pipe_stage_decode()
{
    //If the instruction was flushed
    if (IFtoDE.IFFlush)
    {
        //Clear the flush
        IFtoDE.IFFlush = false;

        //Clear the control values.
        DEtoEX.RegWrite = false;
        DEtoEX.MemtoReg = false;
        DEtoEX.MemRead = false;
        DEtoEX.MemWrite = false;
        DEtoEX.ALUOperation = EXECUTE_NO_OP;
    }

    //The instruction must not have been flushed
    else
    {
        //Disable a previous stall
        PC_Write = true;
        IFtoDE.IFtoDE_Write = true;

        //Decode the instruction

        //The opcode is equal to the lower 6 bits of the hex instruction after it's shifted right 26 bits 
        uint32_t opcode = (0x3f)&IFtoDE.Instruction >> 26;

        //rs is equal to the lower 5 bits of the hex instruction after it's shifted right 21 bits
        uint32_t rs = (0x1f)&(IFtoDE.Instruction >> 21);

        //rt is equal to the lower 5 bits of the hex instruction after it's shifted right 16 bits
        uint32_t rt = (0x1f)&(IFtoDE.Instruction >> 16);

        //rd is equal to the lower 5 bits of the hex instruction after it's shifted right 11 bits
        uint32_t rd = (0x1f)&(IFtoDE.Instruction >> 11);

        //funct is equal to the lower 6 bits of the hex instruction
        uint32_t funct = (0x3f)&(IFtoDE.Instruction);

        //immediate is equal to the lower 16 bits of the hex instruction
        int32_t immediate = (0xffff)&(IFtoDE.Instruction);

        //If the immediate value is negative, sign extend it.
        if ((((immediate >> 15) & 0x1) == 1) && opcode != DECODE_ORI)
        {
            immediate = immediate | 0xffff0000;
        }

        //address is equal to the lower 26 bits of the hex instruction
        uint32_t address = (0x3ffffff)&(IFtoDE.Instruction);

        DEtoEX.rs_Num = rs;
        DEtoEX.rt_Num = rt;
        DEtoEX.rd_Num = rd;

        DEtoEX.immediate = immediate;

        //If a load use hazard occurs
        if (DEtoEX.MemRead && ((DEtoEX.rt_Num == rs) || (DEtoEX.rt_Num == rt)))
        {
            //Insert a stall
            PC_Write = false;
            IFtoDE.IFtoDE_Write = false;

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;
        }

        //Determine which instruction is being performed and set the next stage's regester values appropriately
        //For the R type instructions, the opcode is stored in funct.
        if (opcode == DECODE_FUNCT)
        {
            if (funct == DECODE_ADD)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Not unsigned
                DEtoEX.Unsigned = false;

                //Doing an add operation
                DEtoEX.ALUOperation = EXECUTE_ADD;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;

            }
            else if (funct == DECODE_ADDU)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Is unsigned
                DEtoEX.Unsigned = true;

                //Doing an add operation
                DEtoEX.ALUOperation = EXECUTE_ADD;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;
            }
            else if (funct == DECODE_SUB)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Not unsigned
                DEtoEX.Unsigned = false;

                //Doing a sub operation
                DEtoEX.ALUOperation = EXECUTE_SUB;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;
            }
            else if (funct == DECODE_SUBU)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Is unsigned
                DEtoEX.Unsigned = true;

                //Doing a sub operation
                DEtoEX.ALUOperation = EXECUTE_SUB;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;
            }
            else if (funct == DECODE_SLT)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Not unsigned
                DEtoEX.Unsigned = false;

                //Doing a slt operation
                DEtoEX.ALUOperation = EXECUTE_SLT;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;
            }
            else if (funct == DECODE_SLTU)
            {
                //Writing back to a register
                DEtoEX.RegWrite = true;

                //Not writing memory back to a register
                DEtoEX.MemtoReg = false;

                //Not reading memory
                DEtoEX.MemRead = false;

                //Not writing to memory
                DEtoEX.MemWrite = false;

                //Is unsigned
                DEtoEX.Unsigned = true;

                //Doing a slt operation
                DEtoEX.ALUOperation = EXECUTE_SLT;

                //Using the reg 2 value
                DEtoEX.ALUSrc = false;

                //RS is reg 1
                DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEX.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEX.RegDst = true;
            }
            else
            {                         
                if(IFtoDE.PC != 0)
                {
                    DEtoEX.DataPathStop = true;
                    PC_Write = false;
                    IFtoDE.IFtoDE_Write = false;
                }

                //Clear the control values.
                DEtoEX.RegWrite = false;
                DEtoEX.MemtoReg = false;
                DEtoEX.MemRead = false;
                DEtoEX.MemWrite = false;
                DEtoEX.ALUOperation = EXECUTE_NO_OP;
            }
        }
        else if (opcode == DECODE_LUI)
        {
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Not writing memory back to a register
            DEtoEX.MemtoReg = false;

            //Not reading memory
            DEtoEX.MemRead = false;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Doing a LU operation
            DEtoEX.ALUOperation = EXECUTE_LU;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to RT
            DEtoEX.RegDst = false;
        }
        else if (opcode == DECODE_ORI)
        {
            //Clear the sign extention of the immediate value
            //immediate = immediate & 0xffff;
            
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Not writing memory back to a register
            DEtoEX.MemtoReg = false;

            //Not reading memory
            DEtoEX.MemRead = false;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Doing an or operation
            DEtoEX.ALUOperation = EXECUTE_OR;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEX.RegDst = false;
        }
        else if (opcode == DECODE_ADDI)
        {
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Not writing memory back to a register
            DEtoEX.MemtoReg = false;

            //Not reading memory
            DEtoEX.MemRead = false;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Not unsigned
            DEtoEX.Unsigned = false;

            //Doing a add operation
            DEtoEX.ALUOperation = EXECUTE_ADD;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEX.RegDst = false;
        }
        else if (opcode == DECODE_ADDIU)
        {
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Not writing memory back to a register
            DEtoEX.MemtoReg = false;

            //Not reading memory
            DEtoEX.MemRead = false;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Is unsigned
            DEtoEX.Unsigned = true;

            //Doing a add operation
            DEtoEX.ALUOperation = EXECUTE_ADD;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEX.RegDst = false;

        }
        else if (opcode == DECODE_LW)
        {
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Writing memory back to a register
            DEtoEX.MemtoReg = true;

            //Reading memory
            DEtoEX.MemRead = true;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Doing an mem operation
            DEtoEX.ALUOperation = EXECUTE_MEM;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to RT
            DEtoEX.RegDst = false;
        }
        else if (opcode == DECODE_SW)
        {
            //Not writing back to a register
            DEtoEX.RegWrite = false;

            //Not Reading memory
            DEtoEX.MemRead = false;

            //Writing to memory
            DEtoEX.MemWrite = true;

            //Doing an mem operation
            DEtoEX.ALUOperation = EXECUTE_MEM;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];
        }
        else if (opcode == DECODE_BNE)
        {
            //If the branch is being taken
            if (CURRENT_STATE.REGS[rs] != CURRENT_STATE.REGS[rt])
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDE.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDE.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;

        }
        else if (opcode == DECODE_BEQ)
        {
            //If the branch is being taken
            if (CURRENT_STATE.REGS[rs] == CURRENT_STATE.REGS[rt])
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDE.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDE.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;
        }
        else if (opcode == DECODE_BGTZ)
        {
            //If the branch is being taken
            if ((int)CURRENT_STATE.REGS[rs] > CURRENT_STATE.REGS[0])
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDE.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDE.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;
        }
        else if (opcode == DECODE_SLTI)
        {
            //Writing back to a register
            DEtoEX.RegWrite = true;

            //Not writing memory back to a register
            DEtoEX.MemtoReg = false;

            //Not reading memory
            DEtoEX.MemRead = false;

            //Not writing to memory
            DEtoEX.MemWrite = false;

            //Not unsigned
            DEtoEX.Unsigned = false;

            //Doing a slt operation
            DEtoEX.ALUOperation = EXECUTE_SLT;

            //Using the immediate value
            DEtoEX.ALUSrc = true;

            //RS is reg 1
            DEtoEX.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEX.RegDst = false;
        }
        else if (opcode == DECODE_J)
        {
            //Make the PC take the branch
            Branch = true;

            //Mark the next instruction to be flushed
            IFtoDE.IFFlush = true;

            //Move the program counter to the label.
            Branch_PC = (IFtoDE.PC & 0xf0000000) | (address << 2);

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;

        }
        else
        {
            if (IFtoDE.PC != 0)
            {
                DEtoEX.DataPathStop = true;
                PC_Write = false;
                IFtoDE.IFtoDE_Write = false;
            }

            //Clear the control values.
            DEtoEX.RegWrite = false;
            DEtoEX.MemtoReg = false;
            DEtoEX.MemRead = false;
            DEtoEX.MemWrite = false;
            DEtoEX.ALUOperation = EXECUTE_NO_OP;

        }
    }
    
}

void pipe_stage_fetch()
{
    //Fetch the instruction that the program counter points to.
    uint32_t instruction = mem_read_32(CURRENT_STATE.PC);

    Increament_PC = CURRENT_STATE.PC + 4;

    //If the PC can be modified
    if (PC_Write)
    {
        if (Branch)
        {
            //Make the PC take the branch
            CURRENT_STATE.PC = Branch_PC;

            //Clear the branch
            Branch = false;
        }
        else
        {
            CURRENT_STATE.PC = Increament_PC;
        }
    }

    //If writing to the next stage's regester is enabled
    if(IFtoDE.IFtoDE_Write)
    {
        IFtoDE.PC = Increament_PC;
        IFtoDE.Instruction = instruction;
    }
}

/***************************************************************

 Function: add

 Purpose: Perform a signed addition on the two ALU inputs

 Inputs: 

 reg_1: The first 32 bit ALU input
 
 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t Add(int32_t reg_1, int32_t reg_2)
{
    int32_t result = reg_1 + reg_2;

    //If the result is zero, set the zero flag
    if (result == 0)
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the operation resulted in a number that is greater than the max signed value, or a subtraction that required
	// a borrow into the most significant bit, or a value lowwer than the min signed value, set the carry flag. 
    if ((reg_1 + reg_2 > MAX_SIGNED) || (((reg_1 >= 0) || (reg_2 >= 0)) && (0 > reg_1 + reg_2)) || (reg_1 + reg_2 < MIN_SIGNED))
    {
        CURRENT_STATE.FLAG_C = true;
    }
    else
    {
        CURRENT_STATE.FLAG_C = false;
    }

    //If a sign mismatch occured, set the overflow flag
    if (((reg_1 > 0) && (reg_2 > 0) && (result < 0)) || ((reg_1 < 0) && (reg_2 < 0) && (result > 0)))
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the result is negative, set the negative flag
    if (result < 0)
    {
        CURRENT_STATE.FLAG_N = true;
    }
    else
    {
        CURRENT_STATE.FLAG_N = false;
    }

    return result;
}

/***************************************************************

 Function: addu

 Purpose: Perform a unsigned addition on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t AddU(uint32_t reg_1, uint32_t reg_2)
{
    uint32_t result = reg_1 + reg_2;

    //If the result is zero, set the zero flag
    if (result == 0)
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the operation resulted in a number that is greater than the max signed value, set the carry flag. 
    if (reg_1 + reg_2 > MAX_UNSIGNED)
    {
        CURRENT_STATE.FLAG_C = true;
    }
    else
    {
        CURRENT_STATE.FLAG_C = false;
    }
    
    //Unsigned operations ignore the overflow flag.
    CURRENT_STATE.FLAG_Z = false;
    
    //Unsigned operations can never be negative.
    CURRENT_STATE.FLAG_N = false;

    return result;
}

/***************************************************************

 Function: sub

 Purpose: Perform a signed subtraction on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t Sub(int32_t reg_1, int32_t reg_2)
{
    int32_t result = reg_1 - reg_2;

    //If the result is zero, set the zero flag
    if (result == 0)
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the operation resulted in a number that is greater than the max signed value, or a subtraction that required
    // a borrow into the most significant bit, or a value lowwer than the min signed value, set the carry flag. 
    if ((reg_1 - reg_2 > MAX_SIGNED) || (((reg_1 >= 0) || (reg_2 >= 0)) && (0 > reg_1 + reg_2)) || (reg_1 - reg_2 < MIN_SIGNED))
    {
        CURRENT_STATE.FLAG_C = true;
    }
    else
    {
        CURRENT_STATE.FLAG_C = false;
    }

    //If a sign mismatch occured, set the overflow flag
    if (((reg_1 > 0) && (reg_2 < 0) && (result < 0)) || ((reg_1 < 0) && (reg_2 > 0) && (result > 0)))
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the result is negative, set the negative flag
    if (result < 0)
    {
        CURRENT_STATE.FLAG_N = true;
    }
    else
    {
        CURRENT_STATE.FLAG_N = false;
    }

    return result;
}

/***************************************************************

 Function: subu

 Purpose: Perform a unsigned subtraction on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t SubU(uint32_t reg_1, uint32_t reg_2)
{
    uint32_t result = reg_1 - reg_2;

    //If the result is zero, set the zero flag
    if (result == 0)
    {
        CURRENT_STATE.FLAG_Z = true;
    }
    else
    {
        CURRENT_STATE.FLAG_Z = false;
    }

    //If the operation resulted in a number that is less than zero, set the carry flag. 
    if (reg_1 - reg_2 < 0)
    {
        CURRENT_STATE.FLAG_C = true;
    }
    else
    {
        CURRENT_STATE.FLAG_C = false;
    }

    //Unsigned operations ignore the overflow flag.
    CURRENT_STATE.FLAG_Z = false;

    //Unsigned operations can never be negative.
    CURRENT_STATE.FLAG_N = false;

    return result;
}

/***************************************************************

 Function: slt

 Purpose: Perform a signed set less than on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
int32_t SLT(int32_t reg_1, int32_t reg_2)
{
    //Clear the flags
    CURRENT_STATE.FLAG_Z = false;
    CURRENT_STATE.FLAG_C = false;
    CURRENT_STATE.FLAG_V = false;
    CURRENT_STATE.FLAG_N = false;

    if (reg_1 < reg_2)
    {
        return true;
    }

    return false;
}

/***************************************************************

 Function: sltu

 Purpose: Perform a unsigned set less than on the two ALU inputs

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t SLTU(uint32_t reg_1, uint32_t reg_2)
{
    //Clear the flags
    CURRENT_STATE.FLAG_Z = false;
    CURRENT_STATE.FLAG_C = false;
    CURRENT_STATE.FLAG_V = false;
    CURRENT_STATE.FLAG_N = false;

    if (reg_1 < reg_2)
    {
        return true;
    }

    return false;
}

/***************************************************************

 Function: lu

 Purpose: Perform a load upper on the second ALU input

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t LU(uint32_t reg_2)
{
    uint32_t result = reg_2;
    return (result << 16) & 0xffff0000;
}

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
uint32_t OR(uint32_t reg_1, uint32_t reg_2)
{
     return reg_1 | reg_2;
}

/***************************************************************

 Function: MEM

 Purpose: Calculate the memory address

 Inputs:

 reg_1: The first 32 bit ALU input

 reg_2: The second 32 bit ALU input

 Returns: The 32 bit ALU output.

 Date Modified: March 12nd, 2019
 ***************************************************************/
uint32_t MEM(uint32_t reg_1, int32_t reg_2)
{
    return reg_1 + (reg_2 << 2);
}