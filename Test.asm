addi $t0, $t0, -100
addi $t1, $t2, 100000
addiu $t2, $t1, -100
addiu $t4, $t2, -100
bne $t1, $t2, L1
lw $t6, 0($t2)
L1: bne $zero, $zero, L2
beq $zero, $zero, L2
lw $t6, 0($t2)
L2: beq $t1, $t2, L1
lui $t7, 100
ori $t4, $t2, 100
bgtz $t1, L3
lw $t6, 0($t2)
L3: bgtz $zero, L1
slti $t4, $t0, 0
slti $t4, $t0, -200
j L4
lw $t6, 0($t2)
L4: add $t1, $t2, $t3
addu $t1, $t2, $t3
sub $t1, $t2, $t3
subu $t1, $t2, $t3
slt $t1, $t2, $t3
sltu $t1, $t2, $t3
