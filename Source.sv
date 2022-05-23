//`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////////
//// Company: 
//// Engineer: 
//// 
//// Create Date: 12/26/2021 03:12:32 PM
//// Design Name: 
//// Module Name: program
//// Project Name: 
//// Target Devices: 
//// Tool Versions: 
//// Description: 
//// 
//// Dependencies: 
//// 
//// Revision:
//// Revision 0.01 - File Created
//// Additional Comments:
//// 
////////////////////////////////////////////////////////////////////////////////////



module top(
    output logic dp,
    output logic [3:0] an,
    output logic [6:0] seg,
    input logic clk, btnL, btnR, btnC,
    input logic [11:0] sw
    );
    logic LB, RB, CB;
    logic [3:0] ALUA, ALUB, ALUResult;
    debouncer lbdeb(LB, clk, btnL);
    debouncer rbdeb(RB, clk, btnR);
    debouncer cbdeb(CB, clk, btnC);
    
     controller c(sw,RB,clk,LB,ALUA,ALUB,ALUResult); 
    
     sevenSegmentDisplay display(seg, dp, an, clk, 
                                {1'b0, ALUA}, {1'b0, ALUB}, 5'd17,{1'b0, ALUResult}); 
     endmodule
    

module debouncer(
    output logic btn_out,
    input logic clk, btn_in
    );
    logic [24:0] timer;
    typedef enum logic [1:0]{S0,S1,S2,S3} states;
    states state, nextState;
    logic gotInput;
    
    always_ff@(posedge clk)
        begin    
            state <= nextState;
            if(gotInput)
                timer <= 30000000;
            else
                timer <= timer - 1;
        end
    always_comb
        case(state)
            S0: if(btn_in) 
                begin //startTimer
                    nextState = S1;    
                    gotInput = 1;
                end
                else begin nextState = S0; gotInput = 0; end
            S1: begin nextState = S2; gotInput = 0; end
            S2: begin nextState = S3; gotInput = 0; end
            S3: begin if(timer == 0) nextState = S0; else nextState = S3; gotInput = 0; end
            default: begin nextState = S0; gotInput = 0; end
            endcase
    
    assign btn_out = ( state == S1 );
endmodule

module sevenSegmentDisplay(
    output logic [6:0] seg,
    output logic dp,
    output logic [3:0] an, //active low display
    input logic clk,
    input logic [4:0] in3, in2, in1, in0
    );
    // divide clock by 2^N
    localparam N = 18;
    logic [N-1:0] count = {N{1'b0}}; //initial value 
    
    always_ff @(posedge clk) begin
        count <= count + 1;
    end
    
    logic [4:0] digit_val; // 7-bit register to hold the current data on output 
    logic [3:0] digit_en; //register for enable vector 
    
    always_comb begin
        digit_en = 4'b1111; //default
        digit_val = in0; //default
        case(count[N-1:N-2]) //using only the 2 MSB's of the counter
        
            2'b00 : begin //select first 7Seg.
                digit_val = in0;
                digit_en = 4'b1110;
            end
        
            2'b01: begin //select second 7Seg.
                digit_val = in1;
                digit_en = 4'b1101;
            end
        
            2'b10: begin //select third 7Seg.
                digit_val = in2;
                digit_en = 4'b1011;
            end
            
            2'b11: begin //select forth 7Seg.
                digit_val = in3;
                digit_en = 4'b0111;
            end
        endcase
    end
    
    //Convert digit number to LED vector. LEDs are active low.
    logic [6:0] sseg_LEDs;
    always_comb begin
        sseg_LEDs = 7'b1111111; //default
        case(digit_val)
            5'd0 : sseg_LEDs = 7'b1000000; //to display 0
            5'd1 : sseg_LEDs = 7'b1111001; //to display 1
            5'd2 : sseg_LEDs = 7'b0100100; //to display 2
            5'd3 : sseg_LEDs = 7'b0110000; //to display 3
            5'd4 : sseg_LEDs = 7'b0011001; //to display 4
            5'd5 : sseg_LEDs = 7'b0010010; //to display 5
            5'd6 : sseg_LEDs = 7'b0000010; //to display 6
            5'd7 : sseg_LEDs = 7'b1111000; //to display 7
            5'd8 : sseg_LEDs = 7'b0000000; //to display 8
            5'd9 : sseg_LEDs = 7'b0010000; //to display 9
            5'd10 : sseg_LEDs = 7'b0001000; //to display A
            5'd11 : sseg_LEDs = 7'b0000011; //to display B
            5'd12 : sseg_LEDs = 7'b1000110; //to display C
            5'd13 : sseg_LEDs = 7'b0100001; //to display D
            5'd14 : sseg_LEDs = 7'b0000110; //to display E
            5'd15 : sseg_LEDs = 7'b0001110; //to display F
            5'd16 : sseg_LEDs = 7'b0111111; // to display -
            5'd17 : sseg_LEDs = 7'b0110111; // to display =
            5'd18 : sseg_LEDs = 7'b0100111; // to display c
            default :  sseg_LEDs = 7'b0111111; // display - otherwise
        endcase
    end
    assign an = digit_en;
    assign seg = sseg_LEDs;
    assign dp = 1'b1; //turn dp off
endmodule
     
module controller(input logic [11:0] switches, input logic isEx,clk, rbtn, output logic [3:0] lRead, rRead, result);
    reg [4:0] nextState;
    reg [4:0] curState = 5'b00000;
    reg [2:0] pc_reg; 
    
    assign help = curState;
    
    logic [2:0] RF_add1, RF_add2;
    logic [2:0] RF_wa;
    logic [3:0] RF_wd;
    logic RF_we;
    logic [3:0] RF_d1, RF_d2;
    
    logic [3:0] M_add;
    logic [3:0] M_wd;
    logic  M_we;
    logic [3:0] M_rd;
    
    registerFile RF(RF_add1,RF_add2,RF_wa,RF_wd,clk,RF_we,RF_d1,RF_d2);
    dataMemory DM(M_add,M_wd,clk, M_we,M_rd);
    
    logic enable;
    logic [11:0] IR;
    
    assign enable = 1;
    
    
    instructionMemory IM(pc_reg,enable,IR);
    
    
    logic [2:0] op;
    logic [11:0] ins;
    
    chooseIn choose(switches,IR,isEx,rbtn,clk,ins);
    
    logic inc;
    always_ff@(posedge clk)begin
        
        case(curState)
       
            //init
            5'b00000: 
                begin 
                    pc_reg <= 3'b0000; 
                    curState <= 5'b00001;
                end
             //wait
             5'b00001: 
                begin 
                    if(~(isEx ^ rbtn))
                        begin
                            curState <= 5'b00001;
                        end
//                        else 
//                        curState <= 5'b00010;
                    else if(isEx)
                        begin  
                            curState <= 5'b00010;
                            inc <= 0;    
                        end
                    else if(rbtn)
                        begin
                            curState <= 5'b00010;
                            inc <= 1;
                        end             
                end
                
             //decode
             5'b00010: 
                begin 
                    op = ins[11:9];
                    case(op)
                        3'b000: curState <=  5'b00011;//load
                        3'b001: curState <=  5'b00100; //store
                        3'b010: curState <=  5'b00101; //sub
                        3'b101: curState =  5'b00111; //add
                        3'b111: curState <=  5'b01001;//jump
                       
                    endcase
                   
                end 
                
             //load   
             5'b00011: 
                begin 
                   M_we <= 0; 
                   M_add <= ins[3:0];
                   RF_we = 1;
                   RF_wa = ins[6:4];
                   RF_wd = M_rd;
                   rRead <= M_rd;
                   
                   if(inc)pc_reg <= pc_reg + 1'b1;
                   curState <= 5'b00001;
                end
             //store   
             5'b00100: 
                begin 
                   RF_we <= 0;
                   RF_add1 <= ins[6:4];
                   M_we = 1;
                   M_add = ins[3:0];
                   M_wd = RF_d1;
                   lRead <= RF_d1;
                   if(inc)pc_reg <= pc_reg + 1'b1;
                   curState <= 5'b00001;
                end 
                
             //sub   
             5'b00101: 
                begin 
                     RF_we <=0;
                     RF_add2 <= ins[5:3];
                     RF_add1 <= ins[2:0];
                     curState <= 5'b00110;
                end 
             //sub alu
             5'b00110: 
                begin 
                    RF_we <= 1;
                    RF_wa <= ins[8:6];
                    RF_wd <= RF_d2 - RF_d1; 
                    lRead <= RF_d2;
                    rRead <= RF_d1;
                    result <= RF_d2 - RF_d1;
                    if(inc)pc_reg <= pc_reg + 1'b1;
                    curState <= 5'b00001;
                end 
                
             //add   
             5'b00111: 
                begin 
                   RF_we <= 0;
                   RF_add1 <= ins[5:3];
                   
                   curState <= 5'b01000;
                end 
                
              //add alu  
              5'b01000: 
                begin 
                   lRead = RF_d1;
                   RF_we <= 1;
                   RF_wa <= ins[8:6];
                   rRead = ins[2:0];
                   result = RF_d1 + ins[2:0];
                   RF_wd = RF_d1 + ins[2:0];
                   if(inc)pc_reg <= pc_reg + 1'b1;
                   curState <= 5'b00001;
               end
              //jump
              5'b01001: 
                begin 
                RF_we <= 0;
                RF_add1 = ins[2:0];
                RF_add2 = ins[5:3];
                rRead = RF_d1;
                lRead = RF_d2;
                    if(RF_d1 == RF_d2) begin
                    pc_reg = ((pc_reg + ins[8:6])% 4'b1000);
                    result <= RF_d1 == RF_d2;
                    curState <= 5'b00001;
                end 
                end
                
//              5'b00000: 
//                begin 
//                    pc_reg <= 3'b0000; 
//                    nextState <= (curState + 1);
//                end 
                
//              5'b00000: 
//                begin 
//                    pc_reg <= 3'b0000; 
//                    nextState <= (curState + 1);
//                end  
                
//              //add  
//              5'b00011: 
//                begin 
//                   RF_we <= 0; 
//                   RF_add1 <= ins[5:3];
//                   curState <= 5'b00100;
//                end
                
//             5'b00100: 
//                begin 
//                    RF_we <= 1;
//                    RF_wa <= ins[8:5]; 
//                end 
                  
//              5'b00000: 
//                begin 
//                    pc_reg <= 3'b0000; 
//                    nextState <= (curState + 1);
//                end
//              5'b00000: 
//                begin 
//                    pc_reg <= 3'b0000; 
//                    nextState <= (curState + 1);
//                end
            
              default: curState <= 5'b00000;
        endcase
    end
endmodule


module chooseIn(input logic [11:0] switchesitches, IM, input logic isEx, btn,clk, output logic [11:0] out);
    always_ff@(posedge clk)
        begin
        if(isEx)
           out = switchesitches; 
           else if(btn)
           out <= IM;
        end
endmodule 

module registerFile(
    input logic [2:0] RF_add1, RF_add2,
    input logic [2:0] RF_wa,
    input logic [3:0] RF_wd,
    input logic clk, RF_we,
    output logic [3:0] RF_d1, RF_d2
    );
    
    logic [3:0] RAM [7:0] = {4'b0000, 4'b0000, 4'b0000, 4'b0000,
                             4'b0000, 4'b0000, 4'b0000, 4'b0000}; // initial value
    
   
    always_ff @(posedge clk)
        if (RF_we) RAM[RF_wa] <= RF_wd; // write is alRF_ways synchronous

    always_comb begin
        if(~RF_we)begin
            RF_d1 <= RAM[RF_add1]; // asynchronous read
            RF_d2 <= RAM[RF_add2];
        end
        
    end
endmodule

module dataMemory(
    input logic [3:0] M_add,
    input logic [3:0] M_wd,
    input logic clk, M_we,
    output logic [3:0] M_rd,
    input logic rs
    );
    
    logic [3:0] memory [15:0] = {4'b1111, 4'b1110, 4'b1101, 4'b1100,
                              4'b1011, 4'b1010, 4'b1001, 4'b1000,
                              4'b0111, 4'b0110, 4'b0101, 4'b0100,
                              4'b0011, 4'b0010, 4'b0001, 4'b0000}; 
    
    
    always_ff @(posedge clk)
        if (M_we)
         memory[M_add] <= M_wd; 

    always_comb begin
        if(~M_we)begin
            M_rd <= memory[M_add];
        end
        
        
    end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////
module instructionMemory(input logic [2:0] PC, input logic enable,
                         output logic [11:0] IR);
    logic [11:0] memory [7:0];
    
    always_comb begin
        memory [0] = 12'b111_010_000_000; 
        memory [1] = 12'b000_00_001_1010; 
        memory [2] = 12'b101_000_000_100; 
        memory [3] = 12'b101_001_001_001; 
        memory [4] = 12'b010_000_000_001; 
        memory [5] = 12'b000_00_000_0000; 
        memory [6] = 12'b101_110_001_101; 
        memory [7] = 12'b001_00_110_0000; 
    end
    
     always_comb begin
        if(enable)begin
            case(PC)
                3'b000: IR <= memory[0];
                3'b001: IR <= memory[1];
                3'b010: IR <= memory[2];
                3'b011: IR <= memory[3];
                3'b100: IR <= memory[4];
                3'b101: IR <= memory[5];
                3'b110: IR <= memory[6];
                3'b111: IR <= memory[7];
                default: IR <= memory[0];
            endcase
        end
       
        
    end
  endmodule
  
//module board(
//    input logic clk, btnL, btnR,
//    input logic [11:0] sw,
//    output logic [6:0] seg,
//    output logic dp,
//    output logic [3:0] an
//    );
    
//    logic lBounce, rBounce;
//    debouncer lbdeb(btnL, clk, lBounce);
//    debouncer rbdeb(btnR, clk, rBounce);
    
//     logic [3:0] lDis, rDis, reasultDis;   
//     controller c(sw,rBounce,clk,lBounce,lDis,rDis,reasultDis); 
//     sevenSegmentDisplay dis( clk, {1'b0, lDis}, {1'b0, rDis}, 5'd17,{1'b0, reasultDis},seg, dp, an); 
//     endmodule
    
////this implementation is heavily inspired by online recourses 
//module debouncer(
//    input logic  btn_in,clk,
//    output logic btn_out
//    );
//    logic [24:0] timer;
//    typedef enum logic [1:0]{init,S1,S2,S3} states;
//    states state, nextState;
//    logic empt;
    
//    always_ff@(posedge clk)
//        begin    
//            state <= nextState;
//            if(empt)
//                timer <= timer - 1;
//            else
//                timer <= 25000000;
//        end
//    always_comb
//        case(state)
//            init: if(btn_in) 
//                begin 
//                    nextState = S1;    
//                    empt = 0;
//                end
//                else begin nextState = init; empt = 1; end
//            S1: begin nextState = S2; empt = 1; end
//            S2: begin nextState = S3; empt = 1; end
//            S3: begin if(timer == 0) nextState = init; else nextState = S3; empt = 1; end
//            default: begin nextState = init; empt = 1; end
//            endcase
    
//    assign btn_out = ( state == S1 );
//endmodule

////this implementation is heavily inspired by online recources
//module sevenSegmentDisplay(
//    input logic clk,
//    input logic [4:0] in3, in2, in1, in0,
//    output logic [6:0] seg,
//    output logic dp,
//    output logic [3:0] an
//    );
    
//    logic [17:0] count = {18{1'b0}}; 
    
//    always_ff @(posedge clk) begin
//        count <= count + 1;
//    end
    
    
//    logic [3:0] enc; 
//    logic [4:0] val; 
    
//    always_comb begin
//        enc = 4'b1111; 
//        val = in0; 
//        case(count[17:16]) 
        
//            2'b00 : begin 
//                val = in0;
//                enc = 4'b1110;
//            end
        
//            2'b01: begin 
//                val = in1;
//                enc = 4'b1101;
//            end
        
//            2'b10: begin 
//                val = in2;
//                enc = 4'b1011;
//            end
            
//            2'b11: begin 
//                val = in3;
//                enc = 4'b0111;
//            end
//        endcase
//    end
    
//    //this implementation is heavily inspired by online recources
//    logic [6:0] LEDs;
//    always_comb begin
//        LEDs = 7'b1111111; //default
//        case(val)
//            5'd0 : LEDs = 7'b1000000; 
//            5'd1 : LEDs = 7'b1111001; 
//            5'd2 : LEDs = 7'b0100100; 
//            5'd3 : LEDs = 7'b0110000; 
//            5'd4 : LEDs = 7'b0011001; 
//            5'd5 : LEDs = 7'b0010010; 
//            5'd6 : LEDs = 7'b0000010; 
//            5'd7 : LEDs = 7'b1111000; 
//            5'd8 : LEDs = 7'b0000000; 
//            5'd9 : LEDs = 7'b0010000; 
//            5'd10 : LEDs = 7'b0001000; 
//            5'd11 : LEDs = 7'b0000011; 
//            5'd12 : LEDs = 7'b1000110; 
//            5'd13 : LEDs = 7'b0100001; 
//            5'd14 : LEDs = 7'b0000110; 
//            5'd15 : LEDs = 7'b0001110; 
//            5'd16 : LEDs = 7'b0111111; 
//            5'd17 : LEDs = 7'b0110111; 
//            5'd18 : LEDs = 7'b0100111; 
//            default :  LEDs = 7'b0111111; 
//        endcase
//    end
    
//    assign seg = LEDs;
//    assign an = enc;
//    assign dp = 1'b1; 
    
//endmodule
     
//module controller(input logic [11:0] switches, input logic isEx,clk, rbtn, output logic [3:0] lRead, rRead, result);
//    reg [4:0] nextState;
//    reg [4:0] curState = 5'b00000;
//    reg [2:0] pc_reg; 
    
//    assign help = curState;
    
//    logic [2:0] RF_add1, RF_add2;
//    logic [2:0] RF_wa;
//    logic [3:0] RF_wd;
//    logic RF_we;
//    logic [3:0] RF_d1, RF_d2;
    
//    logic [3:0] M_add;
//    logic [3:0] M_wd;
//    logic  M_we;
//    logic [3:0] M_rd;
    
//    registerFile RF(RF_add1,RF_add2,RF_wa,RF_wd,clk,RF_we,RF_d1,RF_d2);
//    dataMemory DM(M_add,M_wd,clk, M_we,M_rd);
    
//    logic enable;
//    logic [11:0] IR;
    
//    assign enable = 1;
    
    
//    instructionMemory IM(pc_reg,enable,IR);
    
    
//    logic [2:0] op;
//    logic [11:0] ins;
    
//    chooseIn choose(switches,IR,isEx,rbtn,clk,ins);
    
//    logic inc;
//    always_ff@(posedge clk)begin
//        case(curState)
//            //init
//            5'b00000: 
//                begin 
//                    pc_reg <= 3'b0000; 
//                    curState <= 5'b00001;
//                end
//             //wait
//             5'b00001: 
//                begin 
//                    if(~(isEx ^ rbtn))
//                        begin
//                            curState <= 5'b00001;
//                        end
////                        else 
////                        curState <= 5'b00010;
//                    else if(isEx)
//                        begin  
//                            curState <= 5'b00010;
//                            inc = 0;    
//                        end
//                    else if(rbtn)
//                        begin
//                            curState <= 5'b00010;
//                            inc = 1;
//                        end             
//                end
                
//             //decode
//             5'b00010: 
//                begin 
//                    op = ins[11:9];
//                    case(op)
//                        3'b000: curState <=  5'b00011;//load
//                        3'b001: curState <=  5'b00100; //store
//                        3'b010: curState <=  5'b00101; //sub
//                        3'b101: curState =  5'b00111; //add
//                        3'b111: curState <=  5'b01001;//jump
                       
//                    endcase
                   
//                end 
                
//             //load   
//             5'b00011: 
//                begin 
//                   M_we <= 0; 
//                   M_add <= ins[3:0];
//                   RF_we = 1;
//                   RF_wa = ins[6:4];
//                   RF_wd = M_rd;
//                   rRead = M_rd;
//                   curState <= 5'b00001;
//                   if(inc)pc_reg = pc_reg + 1'b1;
//                end
//             //store   
//             5'b00100: 
//                begin 
//                   RF_we <= 0;
//                   RF_add1 <= ins[6:4];
//                   M_we = 1;
//                   M_add = ins[3:0];
//                   M_wd = RF_d1;
//                   lRead <= RF_d1;
//                   if(inc)pc_reg <= pc_reg + 1'b1;
//                   curState <= 5'b00001;
//                end 
                
//             //sub   
//             5'b00101: 
//                begin 
//                     RF_we <=0;
//                     RF_add2 <= ins[5:3];
//                     RF_add1 <= ins[2:0];
//                     curState <= 5'b00110;
//                end 
//             //sub alu
//             5'b00110: 
//                begin 
//                    RF_we <= 1;
//                    RF_wa <= ins[8:6];
//                    RF_wd <= RF_d2 - RF_d1; 
//                    lRead <= RF_d2;
//                    rRead <= RF_d1;
//                    result <= RF_d2 - RF_d1;
//                    if(inc)pc_reg <= pc_reg + 1'b1;
//                    curState <= 5'b00001;
//                end 
                
//             //add   
//             5'b00111: 
//                begin 
//                   RF_we <= 0;
//                   RF_add1 <= ins[5:3];
                   
//                   curState <= 5'b01000;
//                end 
                
//              //add alu  
//              5'b01000: 
//                begin 
//                   lRead = RF_d1;
//                   RF_we <= 1;
//                   RF_wa <= ins[8:6];
//                   rRead = ins[2:0];
//                   result = RF_d1 + ins[2:0];
//                   RF_wd = RF_d1 + ins[2:0];
//                   if(inc)pc_reg <= pc_reg + 1'b1;
//                   curState <= 5'b00001;
//               end
//              //jump
//              5'b01001: 
//                begin 
//                    if(ins[2:0] == ins[5:3])
//                    pc_reg <= (pc_reg + ins[8:6])% 4'b1000;
//                end 
                
////              5'b00000: 
////                begin 
////                    pc_reg <= 3'b0000; 
////                    nextState <= (curState + 1);
////                end 
                
////              5'b00000: 
////                begin 
////                    pc_reg <= 3'b0000; 
////                    nextState <= (curState + 1);
////                end  
                
////              //add  
////              5'b00011: 
////                begin 
////                   RF_we <= 0; 
////                   RF_add1 <= ins[5:3];
////                   curState <= 5'b00100;
////                end
                
////             5'b00100: 
////                begin 
////                    RF_we <= 1;
////                    RF_wa <= ins[8:5]; 
////                end 
                  
////              5'b00000: 
////                begin 
////                    pc_reg <= 3'b0000; 
////                    nextState <= (curState + 1);
////                end
////              5'b00000: 
////                begin 
////                    pc_reg <= 3'b0000; 
////                    nextState <= (curState + 1);
////                end
            
//              default: curState <= 5'b00000;
//        endcase
//    end
//endmodule


//module chooseIn(input logic [11:0] switchler, IM, input logic isEx, btn,clk, output logic [11:0] out);
//    always_ff@(posedge clk)
//        begin
//        if(isEx)
//           out = switchler; 
//           else if(btn)
//           out = IM;
//        end
//endmodule 

//module registerFile(
//    input logic [2:0] RF_add1, RF_add2,
//    input logic [2:0] RF_wa,
//    input logic [3:0] RF_wd,
//    input logic clk, RF_we,
//    output logic [3:0] RF_d1, RF_d2
//    );
    
//    logic [3:0] RAM [7:0] = {4'b0000, 4'b0000, 4'b0000, 4'b0000,
//                             4'b0000, 4'b0000, 4'b0000, 4'b0000}; // initial value
    
//    always_ff @(posedge clk)
//        if (RF_we) RAM[RF_wa] <= RF_wd; 

//    always_comb begin
//        if(~RF_we)begin
//            RF_d1 <= RAM[RF_add1]; 
//            RF_d2 <= RAM[RF_add2];
//        end
//    end
//endmodule

//module dataMemory(
//    input logic [3:0] M_add,
//    input logic [3:0] M_wd,
//    input logic clk, M_we,
//    output logic [3:0] M_rd
//    );
    
//    logic [3:0] memory [15:0] = {4'b1111, 4'b1110, 4'b1101, 4'b1100,
//                              4'b1011, 4'b1010, 4'b1001, 4'b1000,
//                              4'b0111, 4'b0110, 4'b0101, 4'b0100,
//                              4'b0011, 4'b0010, 4'b0001, 4'b0000}; 
    
//    always_ff @(posedge clk)
//        if (M_we)
//         memory[M_add] <= M_wd; 

//    always_comb begin
//        if(~M_we)begin
//            M_rd <= memory[M_add];
//        end
//    end
//endmodule

////////////////////////////////////////////////////////////////////////////////////////////
//module instructionMemory(input logic [2:0] PC, input logic enable,
//                         output logic [11:0] IR);
                         
//    logic [11:0] memory [7:0];
    
//    always_comb begin
//        memory [0] = 12'b111_00_000_0001; // Load RF[0] = DM[8] (= 8)
//        memory [1] = 12'b000_00_001_1010; // Load RF[1] = DM[10] (= 10)
//        memory [2] = 12'b000_00_010_0010; // Load RF[2] = DM[2]  (= 2)
//        memory [3] = 12'b010_011_001_000; // Sub RF[3] = RF[1] - RF[0] (10 - 8 = 2)
//        memory [4] = 12'b111_010_011_010; // Jump PC + 2 if RF[3] == RF[2] (2 == 2)
//        memory [5] = 12'b000_00_000_0000; // Skipped
//        memory [6] = 12'b101_110_001_101; // Add RF[6] = RF[1] + 5 (15 = 10 + 5)
//        memory [7] = 12'b001_00_110_0000; // Store DM[0] = RF[6] (=15)
//    end
//     always_comb begin
//        if(enable)begin
//            case(PC)
//                3'b000: IR <= memory[0];
//                3'b001: IR <= memory[1];
//                3'b010: IR <= memory[2];
//                3'b011: IR <= memory[3];
//                3'b100: IR <= memory[4];
//                3'b101: IR <= memory[5];
//                3'b110: IR <= memory[6];
//                3'b111: IR <= memory[7];
//                default: IR <= memory[0];
//            endcase
//        end
        
//    end
//  endmodule               
      
