module riscv_4bit_processor (
    input clk,
    input reset,
    output [3:0] result
);
    // Internal signals
    wire [3:0] pc;               // Program counter
    wire [11:0] instruction;     // Current instruction
    wire [3:0] rs1_data, rs2_data, alu_result;
    wire [1:0] rs1_addr, rs2_addr, rd_addr;
    wire alu_op;                 // ALU operation from control unit
    wire reg_we;                 // Register write enable from control unit
    
    // Instruction decoding
    assign rs1_addr = instruction[3:2];   // Bits 3:2
    assign rs2_addr = instruction[5:4];   // Bits 5:4
    assign rd_addr = instruction[7:6];    // Bits 7:6
    
    // Instantiate program counter
    program_counter pc_inst (
        .clk(clk),
        .reset(reset),
        .pc(pc)
    );
    
    // Instantiate instruction memory
    instruction_memory imem (
        .pc(pc),
        .instr(instruction)
    );
    
    // Instantiate control unit
    control_unit cu (
        .instruction(instruction),
        .alu_op(alu_op),
        .reg_we(reg_we)
    );
    
    // Instantiate register file
    register_file regfile (
        .clk(clk),
        .reset(reset),
        .we(reg_we),          // Use control unit's write enable
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(rd_addr),
        .rd_data(alu_result),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );
    
    // Instantiate ALU
    alu alu_inst (
        .a(rs1_data),
        .b(rs2_data),
        .opcode(alu_op),      // Use control unit's ALU operation
        .result(alu_result)
    );
    
    // Output result
    assign result = alu_result;
endmodule
