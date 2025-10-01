module riscv_4bit_processor (
    input clk,
    input reset,
    output [3:0] result
);
    wire [3:0] pc;
    wire [11:0] instruction;
    wire [3:0] rs1_data, rs2_data, alu_result;
    wire [1:0] rs1_addr, rs2_addr, rd_addr;
    wire alu_op;
    wire reg_we;
    
    assign rs1_addr = instruction[3:2];
    assign rs2_addr = instruction[5:4];
    assign rd_addr = instruction[7:6];
    
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
