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
 * March 25th, 2019
 *
 **********************************************/

#include "pipe.h"
#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

void pipe_init()
{
    PC_Write = true;

    Branch = false;

    memset(&CURRENT_STATE, 0, sizeof(CPU_State));
    CURRENT_STATE.PC = 0x00400000;

    //Clear input of the IFtoDE reg
    IFtoDEin.PC = 0;
    IFtoDEin.Instruction = 0;
    IFtoDEin.IFFlush = false; 
    IFtoDEin.IFtoDE_Write = false;

    //Clear the input of the DEtoEX reg
    DEtoEXin.RegWrite = false;
    DEtoEXin.MemtoReg = false;
    DEtoEXin.MemRead = false;
    DEtoEXin.MemWrite = false;
    DEtoEXin.RegDst = false;
    DEtoEXin.Unsigned = false;
    DEtoEXin.ALUOperation = 0;
    DEtoEXin.ALUSrc = false;
    DEtoEXin.Reg_1 = 0;
    DEtoEXin.Reg_2 = 0;
    DEtoEXin.immediate = 0;
    DEtoEXin.rs_Num = 0;
    DEtoEXin.rt_Num = 0;
    DEtoEXin.rd_Num = 0;
    DEtoEXin.DataPathStop = false;

    //Clear the EXtoMEM reg
    EXtoMEMin.RegWrite = false;
    EXtoMEMin.MemtoReg = false;
    EXtoMEMin.MemWrite = false;
    EXtoMEMin.MemRead = false;
    EXtoMEMin.ALU_Result = 0;
    EXtoMEMin.Reg_2 = 0;
    EXtoMEMin.Reg_Rd = 0;
    EXtoMEMin.DataPathStop = false;

    //Clear the MEMtoWB reg
    MEMtoWBin.RegWrite = false;
    MEMtoWBin.MemtoReg = false;
    MEMtoWBin.Read_Data = 0;
    MEMtoWBin.ALU_Result = 0;
    MEMtoWBin.Reg_Rd = 0;
    EXtoMEMin.DataPathStop = false;
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
    //Make the MEMtoWB reg output equal to the input
    MEMtoWBout.RegWrite = MEMtoWBin.RegWrite;
    MEMtoWBout.MemtoReg = MEMtoWBin.MemtoReg;
    MEMtoWBout.Read_Data = MEMtoWBin.Read_Data;
    MEMtoWBout.ALU_Result = MEMtoWBin.ALU_Result;
    MEMtoWBout.Reg_Rd = MEMtoWBin.Reg_Rd;
    MEMtoWBout.DataPathStop = MEMtoWBin.DataPathStop;

    if (MEMtoWBout.DataPathStop)
    {
        RUN_BIT = 0;
    }

    if (MEMtoWBout.RegWrite)
    {
        if (MEMtoWBout.MemtoReg)
        {
            CURRENT_STATE.REGS[MEMtoWBout.Reg_Rd] = MEMtoWBout.Read_Data;
            return;
        }

        CURRENT_STATE.REGS[MEMtoWBout.Reg_Rd] = MEMtoWBout.ALU_Result;

    }
}

void pipe_stage_mem()
{
    //Make the EXtoMEM reg output equal to the input
    EXtoMEMout.RegWrite = EXtoMEMin.RegWrite;
    EXtoMEMout.MemtoReg = EXtoMEMin.MemtoReg;
    EXtoMEMout.MemWrite = EXtoMEMin.MemWrite;
    EXtoMEMout.MemRead = EXtoMEMin.MemRead;
    EXtoMEMout.ALU_Result = EXtoMEMin.ALU_Result;
    EXtoMEMout.Reg_2 = EXtoMEMin.Reg_2;
    EXtoMEMout.Reg_Rd = EXtoMEMin.Reg_Rd;
    EXtoMEMout.DataPathStop = EXtoMEMin.DataPathStop;

    //Forwarding values to the next stage
    MEMtoWBin.ALU_Result = EXtoMEMout.ALU_Result;
    MEMtoWBin.Reg_Rd = EXtoMEMout.Reg_Rd;
    MEMtoWBin.RegWrite = EXtoMEMout.RegWrite;
    MEMtoWBin.MemtoReg = EXtoMEMout.MemtoReg;
    MEMtoWBin.DataPathStop = EXtoMEMout.DataPathStop;

    //Data memory segment

    //Reading data from memory?
    if (EXtoMEMout.MemRead)
    {
        MEMtoWBin.Read_Data = mem_read_32(EXtoMEMout.ALU_Result);
    }

    //Writing data to memory?
    else if(EXtoMEMout.MemWrite)
    {
        mem_write_32(EXtoMEMout.ALU_Result, EXtoMEMout.Reg_2);
    }
}

void pipe_stage_execute()
{
    //Make the DEtoEX reg output equal to the input
    DEtoEXout.RegWrite = DEtoEXin.RegWrite;
    DEtoEXout.MemtoReg = DEtoEXin.MemtoReg;
    DEtoEXout.MemRead = DEtoEXin.MemRead;
    DEtoEXout.MemWrite = DEtoEXin.MemWrite;
    DEtoEXout.RegDst = DEtoEXin.RegDst;
    DEtoEXout.Unsigned = DEtoEXin.Unsigned;
    DEtoEXout.ALUOperation = DEtoEXin.ALUOperation;
    DEtoEXout.ALUSrc = DEtoEXin.ALUSrc;
    DEtoEXout.Reg_1 = DEtoEXin.Reg_1;
    DEtoEXout.Reg_2 = DEtoEXin.Reg_2;
    DEtoEXout.immediate = DEtoEXin.immediate;
    DEtoEXout.rs_Num = DEtoEXin.rs_Num;
    DEtoEXout.rt_Num = DEtoEXin.rt_Num;
    DEtoEXout.rd_Num = DEtoEXin.rd_Num;
    DEtoEXout.DataPathStop = DEtoEXin.DataPathStop;

    //ALU operation

    //If an ALU op is going to occur.
    if (DEtoEXout.ALUOperation != EXECUTE_NO_OP)
    {
        //Reg 1 EX forwarding
        if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == DEtoEXout.rs_Num))
        {
            DEtoEXout.Reg_1 = EXtoMEMout.ALU_Result;
        }
        //Reg 1 MEM forwarding
        else if (MEMtoWBout.RegWrite && (MEMtoWBout.Reg_Rd != 0) && (MEMtoWBout.Reg_Rd == DEtoEXout.rs_Num))
        {
            if (MEMtoWBout.MemtoReg)
            {
                DEtoEXout.Reg_1 = MEMtoWBout.Read_Data;
            }

            else
            {
                DEtoEXout.Reg_1 = MEMtoWBout.ALU_Result;
            }
        }

        //Reg 2 EX forwarding
        if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == DEtoEXout.rt_Num))
        {
            DEtoEXout.Reg_2 = EXtoMEMout.ALU_Result;
        }
        //Reg 2 MEM forwarding
        else if (MEMtoWBout.RegWrite && (MEMtoWBout.Reg_Rd != 0) && (MEMtoWBout.Reg_Rd == DEtoEXout.rt_Num))
        {
            if (MEMtoWBout.MemtoReg)
            {
                DEtoEXout.Reg_2 = MEMtoWBout.Read_Data;
            }

            else
            {
                DEtoEXout.Reg_2 = MEMtoWBout.ALU_Result;
            }
        }

        //Forward the second regester value to the next stage.
        EXtoMEMin.Reg_2 = DEtoEXout.Reg_2;

        //Reg 2 value selection
        if (DEtoEXout.ALUSrc)
        {
            DEtoEXout.Reg_2 = DEtoEXout.immediate;
        }


        if (DEtoEXout.ALUOperation == EXECUTE_ADD)
        {
            if (DEtoEXout.Unsigned)
            {
                EXtoMEMin.ALU_Result = AddU(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
            else
            {
                EXtoMEMin.ALU_Result = Add(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
        }

        else if (DEtoEXout.ALUOperation == EXECUTE_SUB)
        {
            if (DEtoEXout.Unsigned)
            {
                EXtoMEMin.ALU_Result = SubU(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
            else
            {
                EXtoMEMin.ALU_Result = Sub(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
        }

        else if (DEtoEXout.ALUOperation == EXECUTE_SLT)
        {
            if (DEtoEXout.Unsigned)
            {
                EXtoMEMin.ALU_Result = SLTU(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
            else
            {
                EXtoMEMin.ALU_Result = SLT(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
            }
        }

        else if (DEtoEXout.ALUOperation == EXECUTE_LU)
        {
            EXtoMEMin.ALU_Result = LU(DEtoEXout.Reg_2);
        }

        else if (DEtoEXout.ALUOperation == EXECUTE_OR)
        {
            EXtoMEMin.ALU_Result = OR(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
        }

        else if (DEtoEXout.ALUOperation == EXECUTE_MEM)
        {
            EXtoMEMin.ALU_Result = MEM(DEtoEXout.Reg_1, DEtoEXout.Reg_2);
        }
    }

    //A no-op must be occuring, clear the ALU result.
    else
    {
        EXtoMEMin.ALU_Result = 0;
    }

    // Forwarding control values
    EXtoMEMin.RegWrite = DEtoEXout.RegWrite;
    EXtoMEMin.MemtoReg = DEtoEXout.MemtoReg;
    EXtoMEMin.MemRead = DEtoEXout.MemRead;
    EXtoMEMin.MemWrite = DEtoEXout.MemWrite;
    EXtoMEMin.DataPathStop = DEtoEXout.DataPathStop;

    //Write-back regester selection
    if (DEtoEXout.RegDst)
    {
        EXtoMEMin.Reg_Rd = DEtoEXout.rd_Num;
    }
    else
    {
        EXtoMEMin.Reg_Rd = DEtoEXout.rt_Num;
    }
    
}

void pipe_stage_decode()
{

    //Make the IFtoDE reg output equal to the input
    IFtoDEout.PC = IFtoDEin.PC;
    IFtoDEout.Instruction = IFtoDEin.Instruction;
    IFtoDEout.IFFlush = IFtoDEin.IFFlush;
    IFtoDEout.IFtoDE_Write = IFtoDEin.IFtoDE_Write;

    //If the instruction was flushed
    if (IFtoDEout.IFFlush)
    {
        //Clear the flush
        IFtoDEin.IFFlush = false;

        //Clear the control values.
        DEtoEXin.RegWrite = false;
        DEtoEXin.MemtoReg = false;
        DEtoEXin.MemRead = false;
        DEtoEXin.MemWrite = false;
        DEtoEXin.ALUOperation = EXECUTE_NO_OP;
    }

    //The instruction must not have been flushed
    else
    {
        //Disable a previous stall
        PC_Write = true;
        IFtoDEin.IFtoDE_Write = true;

        //Decode the instruction

        //The opcode is equal to the lower 6 bits of the hex instruction after it's shifted right 26 bits 
        uint32_t opcode = (0x3f)&IFtoDEout.Instruction >> 26;

        //rs is equal to the lower 5 bits of the hex instruction after it's shifted right 21 bits
        uint32_t rs = (0x1f)&(IFtoDEout.Instruction >> 21);

        //rt is equal to the lower 5 bits of the hex instruction after it's shifted right 16 bits
        uint32_t rt = (0x1f)&(IFtoDEout.Instruction >> 16);

        //rd is equal to the lower 5 bits of the hex instruction after it's shifted right 11 bits
        uint32_t rd = (0x1f)&(IFtoDEout.Instruction >> 11);

        //funct is equal to the lower 6 bits of the hex instruction
        uint32_t funct = (0x3f)&(IFtoDEout.Instruction);

        //immediate is equal to the lower 16 bits of the hex instruction
        int32_t immediate = (0xffff)&(IFtoDEout.Instruction);

        //If the immediate value is negative, sign extend it.
        if ((((immediate >> 15) & 0x1) == 1) && opcode != DECODE_ORI)
        {
            immediate = immediate | 0xffff0000;
        }

        //address is equal to the lower 26 bits of the hex instruction
        uint32_t address = (0x3ffffff)&(IFtoDEout.Instruction);

        DEtoEXin.rs_Num = rs;
        DEtoEXin.rt_Num = rt;
        DEtoEXin.rd_Num = rd;

        DEtoEXin.immediate = immediate;

        //If a load use hazard occurs
        if (DEtoEXout.MemRead && ((EXtoMEMout.Reg_Rd == rs) || (EXtoMEMout.Reg_Rd == rt)))
        {
            //Insert a stall
            PC_Write = false;
            IFtoDEin.IFtoDE_Write = false;

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;

            //Don't let the rest of the funtion change the control values.
            return;
        }

        //Determine which instruction is being performed and set the next stage's regester values appropriately
        //For the R type instructions, the opcode is stored in funct.
        if (opcode == DECODE_FUNCT)
        {
            if (funct == DECODE_ADD)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Not unsigned
                DEtoEXin.Unsigned = false;

                //Doing an add operation
                DEtoEXin.ALUOperation = EXECUTE_ADD;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;

            }
            else if (funct == DECODE_ADDU)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Is unsigned
                DEtoEXin.Unsigned = true;

                //Doing an add operation
                DEtoEXin.ALUOperation = EXECUTE_ADD;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;
            }
            else if (funct == DECODE_SUB)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Not unsigned
                DEtoEXin.Unsigned = false;

                //Doing a sub operation
                DEtoEXin.ALUOperation = EXECUTE_SUB;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;
            }
            else if (funct == DECODE_SUBU)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Is unsigned
                DEtoEXin.Unsigned = true;

                //Doing a sub operation
                DEtoEXin.ALUOperation = EXECUTE_SUB;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;
            }
            else if (funct == DECODE_SLT)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Not unsigned
                DEtoEXin.Unsigned = false;

                //Doing a slt operation
                DEtoEXin.ALUOperation = EXECUTE_SLT;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;
            }
            else if (funct == DECODE_SLTU)
            {
                //Writing back to a register
                DEtoEXin.RegWrite = true;

                //Not writing memory back to a register
                DEtoEXin.MemtoReg = false;

                //Not reading memory
                DEtoEXin.MemRead = false;

                //Not writing to memory
                DEtoEXin.MemWrite = false;

                //Is unsigned
                DEtoEXin.Unsigned = true;

                //Doing a slt operation
                DEtoEXin.ALUOperation = EXECUTE_SLT;

                //Using the reg 2 value
                DEtoEXin.ALUSrc = false;

                //RS is reg 1
                DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

                //RT is reg 2
                DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];

                //Writing back to RD
                DEtoEXin.RegDst = true;
            }
            else
            {                         
                if(IFtoDEout.PC != 0)
                {
                    DEtoEXin.DataPathStop = true;
                    PC_Write = false;
                    IFtoDEin.IFtoDE_Write = false;
                }

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;
            }
        }
        else if (opcode == DECODE_LUI)
        {
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Not writing memory back to a register
            DEtoEXin.MemtoReg = false;

            //Not reading memory
            DEtoEXin.MemRead = false;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Doing a LU operation
            DEtoEXin.ALUOperation = EXECUTE_LU;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to RT
            DEtoEXin.RegDst = false;
        }
        else if (opcode == DECODE_ORI)
        {
            //Clear the sign extention of the immediate value
            //immediate = immediate & 0xffff;
            
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Not writing memory back to a register
            DEtoEXin.MemtoReg = false;

            //Not reading memory
            DEtoEXin.MemRead = false;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Doing an or operation
            DEtoEXin.ALUOperation = EXECUTE_OR;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEXin.RegDst = false;
        }
        else if (opcode == DECODE_ADDI)
        {
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Not writing memory back to a register
            DEtoEXin.MemtoReg = false;

            //Not reading memory
            DEtoEXin.MemRead = false;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Not unsigned
            DEtoEXin.Unsigned = false;

            //Doing a add operation
            DEtoEXin.ALUOperation = EXECUTE_ADD;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEXin.RegDst = false;
        }
        else if (opcode == DECODE_ADDIU)
        {
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Not writing memory back to a register
            DEtoEXin.MemtoReg = false;

            //Not reading memory
            DEtoEXin.MemRead = false;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Is unsigned
            DEtoEXin.Unsigned = true;

            //Doing a add operation
            DEtoEXin.ALUOperation = EXECUTE_ADD;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEXin.RegDst = false;

        }
        else if (opcode == DECODE_LW)
        {
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Writing memory back to a register
            DEtoEXin.MemtoReg = true;

            //Reading memory
            DEtoEXin.MemRead = true;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Doing an mem operation
            DEtoEXin.ALUOperation = EXECUTE_MEM;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to RT
            DEtoEXin.RegDst = false;
        }
        else if (opcode == DECODE_SW)
        {
            //Not writing back to a register
            DEtoEXin.RegWrite = false;

            //Not Reading memory
            DEtoEXin.MemRead = false;

            //Writing to memory
            DEtoEXin.MemWrite = true;

            //Doing an mem operation
            DEtoEXin.ALUOperation = EXECUTE_MEM;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //RT is reg 2
            DEtoEXin.Reg_2 = CURRENT_STATE.REGS[rt];
        }
        else if (opcode == DECODE_BNE)
        {
            uint32_t R1 = CURRENT_STATE.REGS[rs];
            uint32_t R2 = CURRENT_STATE.REGS[rt];

            //Reg 1 EX/MEM delay
            if ((EXtoMEMin.RegWrite && (EXtoMEMin.Reg_Rd != 0) && (EXtoMEMin.Reg_Rd == rs)) ||
                (MEMtoWBin.RegWrite && MEMtoWBin.Read_Data && (MEMtoWBin.Reg_Rd != 0) && (MEMtoWBin.Reg_Rd == rs)))
            {
                //Insert a stall
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;

                //Don't let the rest of the funtion change the control values.
                return;
            }
            //Reg 1 EX forwarding
            else if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == rs))
            {
                R1 = EXtoMEMout.ALU_Result;
            }

            //Reg 2 EX/MEM delay
            if ((EXtoMEMin.RegWrite && (EXtoMEMin.Reg_Rd != 0) && (EXtoMEMin.Reg_Rd == rt)) ||
                (MEMtoWBin.RegWrite && MEMtoWBin.Read_Data && (MEMtoWBin.Reg_Rd != 0) && (MEMtoWBin.Reg_Rd == rt)))
            {
                //Insert a stall
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;

                //Don't let the rest of the funtion change the control values.
                return;
            }

            //Reg 2 EX forwarding
            else if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == rt))
            {
                R2 = EXtoMEMout.ALU_Result;
            }

            //If the branch is being taken
            if (R1 != R2)
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDEin.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDEout.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;

        }
        else if (opcode == DECODE_BEQ)
        {
            uint32_t R1 = CURRENT_STATE.REGS[rs];
            uint32_t R2 = CURRENT_STATE.REGS[rt];

            //Reg 1 EX/MEM delay
            if ((EXtoMEMin.RegWrite && (EXtoMEMin.Reg_Rd != 0) && (EXtoMEMin.Reg_Rd == rs)) ||
                (MEMtoWBin.RegWrite && MEMtoWBin.Read_Data && (MEMtoWBin.Reg_Rd != 0) && (MEMtoWBin.Reg_Rd == rs)))
            {
                //Insert a stall
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;

                //Don't let the rest of the funtion change the control values.
                return;
            }
            //Reg 1 EX forwarding
            else if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == rs))
            {
                R1 = EXtoMEMout.ALU_Result;
            }

            //Reg 2 EX/MEM delay
            if ((EXtoMEMin.RegWrite && (EXtoMEMin.Reg_Rd != 0) && (EXtoMEMin.Reg_Rd == rt)) ||
                (MEMtoWBin.RegWrite && MEMtoWBin.Read_Data && (MEMtoWBin.Reg_Rd != 0) && (MEMtoWBin.Reg_Rd == rt)))
            {
                //Insert a stall
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;

                //Don't let the rest of the funtion change the control values.
                return;
            }

            //Reg 2 EX forwarding
            else if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == rt))
            {
                R2 = EXtoMEMout.ALU_Result;
            }
            
            //If the branch is being taken
            if (R1 == R2)
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDEin.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDEout.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;
        }
        else if (opcode == DECODE_BGTZ)
        {
            uint32_t R1 = CURRENT_STATE.REGS[rs];

            //Reg 1 EX/MEM delay
            if ((EXtoMEMin.RegWrite && (EXtoMEMin.Reg_Rd != 0) && (EXtoMEMin.Reg_Rd == rs)) ||
                (MEMtoWBin.RegWrite && MEMtoWBin.Read_Data && (MEMtoWBin.Reg_Rd != 0) && (MEMtoWBin.Reg_Rd == rs)))
            {
                //Insert a stall
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;

                //Clear the control values.
                DEtoEXin.RegWrite = false;
                DEtoEXin.MemtoReg = false;
                DEtoEXin.MemRead = false;
                DEtoEXin.MemWrite = false;
                DEtoEXin.ALUOperation = EXECUTE_NO_OP;

                //Don't let the rest of the funtion change the control values.
                return;
            }
            //Reg 1 EX forwarding
            else if (EXtoMEMout.RegWrite && (EXtoMEMout.Reg_Rd != 0) && (EXtoMEMout.Reg_Rd == rs))
            {
                R1 = EXtoMEMout.ALU_Result;
            }

            //If the branch is being taken
            if ((int)R1 > CURRENT_STATE.REGS[0])
            {
                //Make the PC take the branch
                Branch = true;

                //Mark the next instruction to be flushed
                IFtoDEin.IFFlush = true;

                //Make the PC point to the branch target
                Branch_PC = IFtoDEout.PC + (immediate << 2);
            }

            else
            {
                //Make the PC not take the branch
                Branch = false;
            }

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;
        }
        else if (opcode == DECODE_SLTI)
        {
            //Writing back to a register
            DEtoEXin.RegWrite = true;

            //Not writing memory back to a register
            DEtoEXin.MemtoReg = false;

            //Not reading memory
            DEtoEXin.MemRead = false;

            //Not writing to memory
            DEtoEXin.MemWrite = false;

            //Not unsigned
            DEtoEXin.Unsigned = false;

            //Doing a slt operation
            DEtoEXin.ALUOperation = EXECUTE_SLT;

            //Using the immediate value
            DEtoEXin.ALUSrc = true;

            //RS is reg 1
            DEtoEXin.Reg_1 = CURRENT_STATE.REGS[rs];

            //Writing back to Rt
            DEtoEXin.RegDst = false;
        }
        else if (opcode == DECODE_J)
        {
            //Make the PC take the branch
            Branch = true;

            //Mark the next instruction to be flushed
            IFtoDEin.IFFlush = true;

            //Move the program counter to the label.
            Branch_PC = (IFtoDEout.PC & 0xf0000000) | (address << 2);

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;

        }
        else
        {
            if (IFtoDEout.PC != 0)
            {
                DEtoEXin.DataPathStop = true;
                PC_Write = false;
                IFtoDEin.IFtoDE_Write = false;
            }

            //Clear the control values.
            DEtoEXin.RegWrite = false;
            DEtoEXin.MemtoReg = false;
            DEtoEXin.MemRead = false;
            DEtoEXin.MemWrite = false;
            DEtoEXin.ALUOperation = EXECUTE_NO_OP;

        }
    }
    
}

void pipe_stage_fetch()
{
    //Fetch the instruction that the program counter points to.
    uint32_t instruction = mem_read_32(CURRENT_STATE.PC);

    //The PC after being inceamented by 4
    uint32_t Increament_PC = CURRENT_STATE.PC + 4;

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
    if(IFtoDEin.IFtoDE_Write)
    {
        IFtoDEin.PC = Increament_PC;
        IFtoDEin.Instruction = instruction;
    }
}


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


uint32_t LU(uint32_t reg_2)
{
    uint32_t result = reg_2;
    return (result << 16) & 0xffff0000;
}


uint32_t OR(uint32_t reg_1, uint32_t reg_2)
{
     return reg_1 | reg_2;
}


uint32_t MEM(uint32_t reg_1, int32_t reg_2)
{
    return reg_1 + (reg_2 << 2);
}