module alu (
    input [3:0] a,
    input [3:0] b,
    input opcode,
    output reg [3:0] result
);
    always @(*) begin
        case (opcode)
            1'b0: result = a + b;
            1'b1: result = a - b;
            default: result = 4'b0;
        endcase
    end
endmodule

module control_unit (
    input [3:0] instruction,
    output reg alu_op,
    output reg reg_we
);
    wire opcode;
    
    assign opcode = instruction[3];
    
    always @(*) begin
        alu_op = 1'b0;
        reg_we = 1'b0;
        
        case (opcode)
            1'b0: begin
                alu_op = 1'b0;
                reg_we = 1'b1;
            end
            1'b1: begin
                alu_op = 1'b1;
                reg_we = 1'b1;
            end
            default: begin
                alu_op = 1'b0;
                reg_we = 1'b0;
            end
        endcase
    end
endmodule

module program_counter (
    input clk,
    input reset,
    output reg [3:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 4'b0000;
        end
        else begin
            pc <= pc + 1;
        end
    end
endmodule

module register_file (
    input clk,
    input reset,
    input we,
    input [1:0] rs1_addr,
    input [1:0] rs2_addr,
    input [1:0] rd_addr,
    input [3:0] rd_data,
    output [3:0] rs1_data,
    output [3:0] rs2_data
);
    reg [3:0] registers [0:3];
    
    assign rs1_data = registers[rs1_addr];
    assign rs2_data = registers[rs2_addr];
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            registers[0] <= 4'd2;
            registers[1] <= 4'd0;
            registers[2] <= 4'd3;
            registers[3] <= 4'd0;
        end
        else if (we) begin
            registers[rd_addr] <= rd_data;
        end
    end
endmodule

module instruction_memory (
    input [3:0] pc,
    output reg [3:0] instr
);
    reg [3:0] mem [0:15];
    
    initial begin
        mem[0] = 4'b0000;
        mem[1] = 4'b0010;
        mem[2] = 4'b1100;
        mem[3] = 4'b1000;
        mem[4] = 4'b0000;
        mem[5] = 4'b0000;
        mem[6] = 4'b0000;
        mem[7] = 4'b0000;
        mem[8] = 4'b0000;
        mem[9] = 4'b0000;
        mem[10] = 4'b0000;
        mem[11] = 4'b0000;
        mem[12] = 4'b0000;
        mem[13] = 4'b0000;
        mem[14] = 4'b0000;
        mem[15] = 4'b0000;
    end
    
    always @(*) begin
        instr = mem[pc];
    end
endmodule

module riscv_4bit_processor (
    input clk,
    input reset,
    output [3:0] result
);
    wire [3:0] pc;
    wire [3:0] instruction;
    wire [3:0] rs1_data, rs2_data, alu_result;
    wire [1:0] rs1_addr, rs2_addr, rd_addr;
    wire alu_op;
    wire reg_we;
    
    assign rd_addr = instruction[2:1];
    assign rs1_addr = rd_addr;
    assign rs2_addr = instruction[0] ? 2'b01 : 2'b00;
    
    program_counter pc_inst (
        .clk(clk),
        .reset(reset),
        .pc(pc)
    );
    
    instruction_memory imem (
        .pc(pc),
        .instr(instruction)
    );
    
    control_unit cu (
        .instruction(instruction),
        .alu_op(alu_op),
        .reg_we(reg_we)
    );
    
    register_file regfile (
        .clk(clk),
        .reset(reset),
        .we(reg_we),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(rd_addr),
        .rd_data(alu_result),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );
    
    alu alu_inst (
        .a(rs1_data),
        .b(rs2_data),
        .opcode(alu_op),
        .result(alu_result)
    );
    
    assign result = alu_result;
endmodule

module riscv_4bit_processor_tb;
    reg clk;
    reg reset;
    wire [3:0] result;
    
    riscv_4bit_processor dut (
        .clk(clk),
        .reset(reset),
        .result(result)
    );
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    initial begin
        $dumpfile("processor.vcd");
        $dumpvars(0, riscv_4bit_processor_tb);
        
        reset = 1;
        #10;
        reset = 0;
        
        @(posedge clk);
        repeat(3) @(posedge clk);
        #1;
        
        $display("Final Result: %d", result);
        
        if (result === 4'd0)
            $display("Test Passed!");
        else
            $display("Test Failed! Expected 0, got %d", result);
            
        $finish;
    end
    
    initial begin
        $monitor("Time = %0t: PC = %d, Instr = %b, r0 = %d, r1 = %d, r2 = %d, r3 = %d, ALU Result = %d",
                 $time, dut.pc, dut.instruction, 
                 dut.regfile.registers[0], dut.regfile.registers[1], 
                 dut.regfile.registers[2], dut.regfile.registers[3], 
                 result);
    end
endmodule
