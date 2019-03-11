/**********************************************
 *
 * Program Title: Lab_2
 *
 * Program File Name: sim.c
 *
 * Microprocessors II EECE.4800-201
 *
 * Spring 2019
 *
 * Authors: Justice Graves and Colin Rockwood
 *
 * February 22nd, 2019
 *
 **********************************************/

#include <stdio.h>
#include "shell.h"

// Defintions of opcodes for instructions that are NOT functions
#define FUNCT	0x00
// Definitions of opcodes for instructions that ARE functions
#define ADD	0x20
#define ADDU 0x21
#define SUB	0x22
#define SUBU 0x23
#define SLT	0x2a
#define SLTU 0x2b
#define LUI	0x0F
#define ORI	0x0d
#define ADDI 0x08
#define ADDIU 0x09
#define LW	0x23
#define SW	0x2b
#define LB	0x20
#define SB	0x28
#define BNE	0x05
#define BEQ	0x04
#define BGTZ 0x07
#define SLTI 0x0a
#define J 0x02
#define JAL 0x03

//The max value that an unsigned 32 bit int can hold
#define MAX_UNSIGNED 0xffffffff

uint32_t instruction;
uint32_t opcode;
uint32_t rs;
uint32_t rt;
uint32_t rd;
uint32_t shamt;
uint32_t funct;
int32_t immediate;
uint32_t address;

//R-Type instructions
void add();
void addu();
void sub();
void subu();
void slt();
void sltu();

//I-type instructions
void lui();
void ori();
void addi();
void addiu();
void lw();
void sw();
void lb();
void sb();
void bne();
void beq();
void bgtz();
void slti();

//J-Type instructions
void j();
void jal();

 /***************************************************************

 Function: fetch

 Purpose: Fetch and store the instruction that will be executed

 Inputs: None

 Returns: None

 Date Modified: February 17th, 2019
 ***************************************************************/
void fetch()
{
	//Fetch the instruction that the program counter points to.
	instruction = mem_read_32(CURRENT_STATE.PC);
}

/***************************************************************

 Function: decode

 Purpose: Decode the stored instruction

 Inputs: None

 Returns: None

 Date Modified: February 17th, 2019
 ***************************************************************/
void decode()
{
	//The opcode is equal to the lower 6 bits of the hex instruction after it's shifted right 26 bits 
	opcode = (0x3f)&instruction >> 26;

	//rs is equal to the lower 5 bits of the hex instruction after it's shifted right 21 bits
	rs = (0x1f)&(instruction >> 21);

	//rt is equal to the lower 5 bits of the hex instruction after it's shifted right 16 bits
	rt = (0x1f)&(instruction >> 16);

	//rd is equal to the lower 5 bits of the hex instruction after it's shifted right 11 bits
	rd = (0x1f)&(instruction >> 11);

	//shamt is equal to the lower 5 bits of the hex instruction after it's shifted right 6 bits
	shamt = (0x1f)&(instruction >> 6);
	
	//funct is equal to the lower 6 bits of the hex instruction
	funct = (0x3f)&(instruction);
	
	//immediate is equal to the lower 16 bits of the hex instruction
	immediate = (0xffff)&(instruction);
	
	//If the immediate value is negative, sign extend it.
	if (((immediate >> 15) & 0x1) == 1)
		immediate = immediate | 0xffff0000;
	
	//address is equal to the lower 26 bits of the hex instruction
	address = (0x3ffffff)&(instruction);
	
}

/***************************************************************

 Function: execute

 Purpose: Execute the decoded instruction

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void execute()
{
	//For the R type instructions, the opcode is stored in funct.
	if (opcode == FUNCT)
	{
		if (funct == ADD)
			add();
		else if (funct == ADDU)
			addu();
		else if (funct == SUB)
			sub();
		else if (funct == SUBU)
			subu();
		else if (funct == SLT)
			slt();
		else if (funct == SLTU)
			sltu();
		else
			RUN_BIT = 0;
	}
	else if (opcode == LUI)
		lui();
	else if (opcode == ORI)
		ori();
	else if (opcode == ADDI)
		addi();
	else if (opcode == ADDIU)
		addiu();
	else if (opcode == LW)
		lw();
	else if (opcode == SW)
		sw();
	else if (opcode == LB)
		lb();
	else if (opcode == SB)
		sb();
	else if (opcode == BNE)
		bne();
	else if (opcode == BEQ)
		beq();
	else if (opcode == BGTZ)
		bgtz();
	else if (opcode == SLTI)
		slti();
	else if (opcode == J)
		j();
	else if (opcode == JAL)
		jal();
	else
		RUN_BIT = 0;
}

/***************************************************************

 Function: process_instruction

 Purpose: Process the next instruction and update the simulated architecture state

 Inputs: None

 Returns: None

 Date Modified: February 20th, 2019
 ***************************************************************/
void process_instruction()
{
	//So that we only need to change the values relevant to the instruction being performed
	NEXT_STATE = CURRENT_STATE;

	//Clear all of the flags, the instructions will set them if they need to.
	NEXT_STATE.FLAG_N = 0;
	NEXT_STATE.FLAG_Z = 0;
	NEXT_STATE.FLAG_V = 0;
	NEXT_STATE.FLAG_C = 0;

	//Fetch, decode, and execute the instruction
    fetch();
    decode();
    execute();
	
}

/***************************************************************

 Function: add

 Purpose: Perform a signed addition on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void add()
{
	//rd = rs - rt
	NEXT_STATE.REGS[rd] = (int32_t)CURRENT_STATE.REGS[rs] + (int32_t)CURRENT_STATE.REGS[rt];

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rd] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the instruction resulted in a value that is greater than the max value that an unsigned regester can hold, or a subtraction that required
	// a borrow into the most significant, set the carry flag.
	if ((MAX_UNSIGNED < (int32_t)CURRENT_STATE.REGS[rs] + (int32_t)CURRENT_STATE.REGS[rt]) ||
		((((int32_t)CURRENT_STATE.REGS[rs] >= 0) || ((int32_t)CURRENT_STATE.REGS[rt] >= 0)) && (0 > (int32_t)CURRENT_STATE.REGS[rs] + (int32_t)CURRENT_STATE.REGS[rt])))
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//If a sign mis-match occurred, set the overflow flag.
	if ((((int32_t)NEXT_STATE.REGS[rs] > 0) && ((int32_t)CURRENT_STATE.REGS[rt] > 0) && (0 > (int32_t)NEXT_STATE.REGS[rs] + (int32_t)CURRENT_STATE.REGS[rt])) ||
		(((int32_t)NEXT_STATE.REGS[rs] < 0) && ((int32_t)CURRENT_STATE.REGS[rt] < 0) && (0 < (int32_t)NEXT_STATE.REGS[rs] + (int32_t)CURRENT_STATE.REGS[rt])))
	{
		NEXT_STATE.FLAG_V = 1;
	}

	//If the result is negative, set the negative flag.
	if ((int32_t)NEXT_STATE.REGS[rd] < 0)
	{
		NEXT_STATE.FLAG_N = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: addu

 Purpose: Perform a unsigned addition on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void addu()
{
	NEXT_STATE.REGS[rd] = CURRENT_STATE.REGS[rs] + CURRENT_STATE.REGS[rt];

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rd] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the instruction resulted in a value that is greater than the max value that an unsigned regester can hold, set the carry flag.
	if ((MAX_UNSIGNED < CURRENT_STATE.REGS[rs] + CURRENT_STATE.REGS[rt]))
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: sub

 Purpose: Perform a signed subtraction on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void sub()
{
	//rd = rs - rt
	NEXT_STATE.REGS[rd] = (int32_t)CURRENT_STATE.REGS[rs] - (int32_t)CURRENT_STATE.REGS[rt];

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rd] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the instruction resulted in a value that is greater than the max value that an unsigned regester can hold, or a subtraction that required
	// a borrow into the most significant, set the carry flag.
	if ((MAX_UNSIGNED < (int32_t)CURRENT_STATE.REGS[rs] - (int32_t)CURRENT_STATE.REGS[rt]) ||
		((((int32_t)CURRENT_STATE.REGS[rs] >= 0) || ((int32_t)CURRENT_STATE.REGS[rt] >= 0)) && (0 > (int32_t)CURRENT_STATE.REGS[rs] - (int32_t)CURRENT_STATE.REGS[rt])))
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//If a sign mis-match occurred, set the overflow flag.
	if ((((int32_t)NEXT_STATE.REGS[rs] > 0) && ((int32_t)CURRENT_STATE.REGS[rt] > 0) && (0 > (int32_t)NEXT_STATE.REGS[rs] - (int32_t)CURRENT_STATE.REGS[rt])) ||
		(((int32_t)NEXT_STATE.REGS[rs] < 0) && ((int32_t)CURRENT_STATE.REGS[rt] < 0) && (0 < (int32_t)NEXT_STATE.REGS[rs] - (int32_t)CURRENT_STATE.REGS[rt])))
	{
		NEXT_STATE.FLAG_V = 1;
	}

	//If the result is negative, set the negative flag.
	if ((int32_t)NEXT_STATE.REGS[rd] < 0)
	{
		NEXT_STATE.FLAG_N = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: subu

 Purpose: Perform a unsigned subtraction on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void subu()
{
	//rd = rs - rt
	NEXT_STATE.REGS[rd] = CURRENT_STATE.REGS[rs] - CURRENT_STATE.REGS[rt];

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rd] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the operation results in a number that is negative, set the carry bit.
	if ((int32_t)(CURRENT_STATE.REGS[rs] - CURRENT_STATE.REGS[rt]) < 0)
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: slt

 Purpose: Perform a singed set less than on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void slt()
{
	if ((int32_t)CURRENT_STATE.REGS[rs] < (int32_t)CURRENT_STATE.REGS[rt])	// when rs is less than rt, set the result to "TRUE"
		NEXT_STATE.REGS[rd] = 1;
	else							// when rs is not less than rt, set the result to "FALSE"
		NEXT_STATE.REGS[rd] = 0;

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: sltu

 Purpose: Perform a unsigned set less than on two registers instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void sltu()
{
	if(CURRENT_STATE.REGS[rs] < CURRENT_STATE.REGS[rt])	// when rs is less than rt, set the result to "TRUE"
		NEXT_STATE.REGS[rd] = 1;
	else							// when rs is not less than rt, set the result to "FALSE"
		NEXT_STATE.REGS[rd] = 0;

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: lui

 Purpose: Perform a load upper immediate instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void lui()
{
	//The upper 16 bits of RT equals the immediate value
	NEXT_STATE.REGS[rt] = immediate;
	NEXT_STATE.REGS[rt] = (NEXT_STATE.REGS[rt] << 16) & 0xffff0000;
	
	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: ori

 Purpose: Perform a logical OR with an immediate value instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void ori()
{
	//Or the RS value with the immediate value and store it in RT
	NEXT_STATE.REGS[rt] = CURRENT_STATE.REGS[rs] | (immediate&0x0000ffff);
	
	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: addi

 Purpose: Perform a signed add with an immediate value instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void addi()
{
	NEXT_STATE.REGS[rt] = (int32_t)CURRENT_STATE.REGS[rs] + immediate;

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the instruction resulted in a value that is greater than the max value that an unsigned regester can hold, or a subtraction that required
	// a borrow into the most significant, set the carry flag.
	if ((MAX_UNSIGNED < (int32_t)CURRENT_STATE.REGS[rs] + immediate) ||
		((((int32_t)CURRENT_STATE.REGS[rs] >= 0) || (immediate >= 0)) && (0 > (int32_t)CURRENT_STATE.REGS[rs] + immediate)))
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//If a sign mis-match occurred, set the overflow flag.
	if ((((int32_t)NEXT_STATE.REGS[rs] > 0) && (immediate > 0) && (0 > (int32_t)NEXT_STATE.REGS[rs] + immediate)) || 
		(((int32_t)NEXT_STATE.REGS[rs] < 0) && (immediate < 0) && (0 < (int32_t)NEXT_STATE.REGS[rs] + immediate)))
	{
		NEXT_STATE.FLAG_V = 1;
	}

	//If the result is negative, set the negative flag.
	if ((int32_t)NEXT_STATE.REGS[rt] < 0)
	{
		NEXT_STATE.FLAG_N = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;

}

/***************************************************************

 Function: addui

 Purpose: Perform an unsigned add with an immediate value instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void addiu()
{
	NEXT_STATE.REGS[rt] = NEXT_STATE.REGS[rs] + (uint32_t)immediate;

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the instruction resulted in a value that is greater than the max value that an unsigned regester can hold, set the carry flag.
	if ((MAX_UNSIGNED < NEXT_STATE.REGS[rs] + (uint32_t)immediate))
	{
		NEXT_STATE.FLAG_C = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: lw

 Purpose: Perform a load word instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
 ***************************************************************/
void lw()
{
	//RT equals the data stored at the address stored at rs + immediate.
	NEXT_STATE.REGS[rt] = mem_read_32((CURRENT_STATE.REGS[rs] + (immediate*4)));

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the result is negative, set the negative flag.
	if ((int32_t)NEXT_STATE.REGS[rt] < 0)
	{
		NEXT_STATE.FLAG_N = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: sw

 Purpose: Perform a store word instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void sw()
{
	//Get the word at the address + offset
	NEXT_STATE.REGS[rt] = mem_read_32((CURRENT_STATE.REGS[rs] + (immediate * 4)));

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: lb

 Purpose: Perform a load byte instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22th, 2019
 ***************************************************************/
void lb()
{
	//Does the address line up with the 32 bit addresses?
	if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 0)
	{
		//RT equals the lower 8 bits of data stored at the address.
		NEXT_STATE.REGS[rt] = mem_read_32((CURRENT_STATE.REGS[rs] + (immediate))) & 0xff;
	}
	//Does the address line up with the one above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 1)
	{
		//RT equals the lower 8 bits of data stored at the address - 1 after it's been shifted right two.
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate-1)))>>2) & 0xff;
	}
	//Does the address line up with the two above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 2)
	{
		//RT equals the lower 8 bits of data stored at the address - 2 after it's been shifted right four.
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate - 2))) >> 4) & 0xff;
	}
	//Does the address line up with the three above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 3)
	{
		//RT equals the lower 8 bits of data stored at the address - 3 after it's been shifted right six.
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate - 3))) >> 6) & 0xff;
	}

	//If the instruction resulted in a zero value, set the zero flag.
	if (NEXT_STATE.REGS[rt] == 0)
	{
		NEXT_STATE.FLAG_Z = 1;
	}

	//If the result is negative, set the negative flag.
	if ((int32_t)NEXT_STATE.REGS[rt] < 0)
	{
		NEXT_STATE.FLAG_N = 1;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}

/***************************************************************

 Function: sb

 Purpose: Perform a store byte instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void sb()
{
	//Does the address line up with the 32 bit addresses?
	if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 0)
	{
		//Get the word at the address+offset and set it's lower 8 bits equal to the lower 8 bits of RT
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate))) & 0xffffff00) | (CURRENT_STATE.REGS[rt] & 0xff);
	}
	//Does the address line up with the one above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 1)
	{
		//Get the word at the address+offset and set it's middle lower 8 bits equal to the lower 8 bits of RT
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate))) & 0xffff00ff) | ((CURRENT_STATE.REGS[rt] & 0xff)<<2);
	}
	//Does the address line up with the two above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 2)
	{
		//Get the word at the address+offset and set it's middle upper 8 bits equal to the lower 8 bits of RT
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate))) & 0xff00ffff) | ((CURRENT_STATE.REGS[rt] & 0xff) << 4);
	}
	//Does the address line up with the three above the 32 bit addresses?
	else if (CURRENT_STATE.REGS[rs] + (immediate) % 4 == 3)
	{
		//Get the word at the address+offset and set it's upper 8 bits equal to the lower 8 bits of RT
		NEXT_STATE.REGS[rt] = (mem_read_32((CURRENT_STATE.REGS[rs] + (immediate))) & 0x00ffffff) | ((CURRENT_STATE.REGS[rt] & 0xff) << 6);
	}


	//Write the modified word back into memory
	mem_write_32((CURRENT_STATE.REGS[rs] + (immediate)), CURRENT_STATE.REGS[rt]);

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC + 4;
}

/***************************************************************

 Function: bne

 Purpose: Perform a branch not equal instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void bne()
{
	if (CURRENT_STATE.REGS[rs] != CURRENT_STATE.REGS[rt])
	{
		//Move the program counter to the label.
		NEXT_STATE.PC = CURRENT_STATE.PC + (immediate<<2) + 4;
	}
	else
	{
		//Move the program counter to the next instruction.
		NEXT_STATE.PC = CURRENT_STATE.PC+4;
	}
}

/***************************************************************

 Function: beq

 Purpose: Perform a branch equal instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void beq()
{
	if (CURRENT_STATE.REGS[rs] == CURRENT_STATE.REGS[rt])
	{
		//Move the program counter to the label.
		NEXT_STATE.PC = CURRENT_STATE.PC + (immediate<<2) + 4;
	}
	else
	{
		//Move the program counter to the next instruction.
		NEXT_STATE.PC = CURRENT_STATE.PC+4;
	}
}

/***************************************************************

 Function: bgtz

 Purpose: Perform a branch greater than zero instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void bgtz()
{

	if ((int32_t)CURRENT_STATE.REGS[rs] > CURRENT_STATE.REGS[0])
	{
		//Move the program counter to the label.
		NEXT_STATE.PC = CURRENT_STATE.PC + (immediate<<2);
	}
	else
	{
		//Move the program counter to the next instruction.
		NEXT_STATE.PC = CURRENT_STATE.PC+4;
	}
}

/***************************************************************

 Function: slti

 Purpose: Perform a set less than immediate instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void slti()
{
	if ((int32_t)CURRENT_STATE.REGS[rs] < immediate)
	{
		NEXT_STATE.REGS[rt] = 1;
	}
	else
	{
		NEXT_STATE.REGS[rt] = 0;
	}

	//Move the program counter to the next instruction.
	NEXT_STATE.PC = CURRENT_STATE.PC+4;
}

/***************************************************************

 Function: j

 Purpose: Perform a unconditional jump instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void j()
{
	//Move the program counter to the label.
	NEXT_STATE.PC = (CURRENT_STATE.PC & 0xf0000000) | (address<<2);
}

/***************************************************************

 Function: jal

 Purpose: Perform a unconditional jump and link instruction in MIPS architecture

 Inputs: None

 Returns: None

 Date Modified: February 22nd, 2019
***************************************************************/
void jal()
{
	//Store the current PC
	NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;

	//Move the program counter to the label.
	NEXT_STATE.PC = (CURRENT_STATE.PC & 0xf0000000) | (address << 2);
}