# RISC
A simple reduce instuctions set computer which allows several simple operations using 1 memory and a register file. 
Operations availabe: loading from memory, storing to memory, addition, subtraction, conditional branch.


-How to use: 
1-Import source code and constraint file in vivado v.2020+
2- generate bitstream
3-Connect to Basys3 and code it
4- LSB of switched represents the instruction
5- generate the proper insturction
6-press right button to do the instruction

-the following insturctions will be done in order by pressing the left button:

// Load RF[1] = DM[10] (= 10)
// Load RF[2] = DM[2]  (= 2)
// Sub RF[3] = RF[1] - RF[0] (10 - 8 = 2)
// Jump PC + 2 if RF[3] == RF[2] (2 == 2)
// Add RF[6] = RF[1] + 5 (15 = 10 + 5)
// Store DM[0] = RF[6] (=15)

-the format of instructions:

  Load –000 xx r2r1r0 d3d2d1d0: This instruction specifies a move of data from the data 
  memory location (D) whose address is specified by bits [d3d2d1d0] into the register file (RF) 
  whose address location is specified by the bits [r2r1r0].

  Store –001 xx r2r1r0 d3d2d1d0: This instruction specifies a move of data in the opposite
  direction as the instruction load, meaning a move of data from the register file to the data 
  memory

  Sub– 010 wa2wa1wa0 rb2rb1rb0 ra2ra1ra0: This instruction specifies a subtraction of two
  register-file specified by [rb2rb1rb0] and [ra2ra1ra0], with the result stored in the register 
  file specified by [wa2wa1wa0].

  Add constant – 101 wa2wa1wa0 rb2rb1rb0 ca2ca1ca0 : This instruction specifies a sum 
  operation of one register-file specified by [rb2rb1rb0] and constant value ca2ca1ca0, with 
  the result stored in the register file specified by [wa2wa1wa0]

  Jump - 111 ca2ca1ca0 rb2rb1rb0 ra2ra1ra0 : This instruction is a simple representation of
  conditional statement in C programming language. It compares thevalue of the registers in 
  the register-file specified by [rb2rb1rb0] and [ra2ra1ra0]
