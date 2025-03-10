module riscv_4bit_processor (
    input clk,
    input reset,
    output [3:0] result
);
    // Internal signals
    reg [3:0] pc;
    wire [11:0] instruction;
    wire [3:0] rs1_data, rs2_data, alu_result;
    wire [1:0] rs1_addr, rs2_addr, rd_addr;
    wire opcode;
    
    // Instruction decoding
    assign opcode = instruction[0];       // Bit 0: 0=ADD, 1=SUB
    assign rs1_addr = instruction[3:2];   // Bits 3:2
    assign rs2_addr = instruction[5:4];   // Bits 5:4
    assign rd_addr = instruction[7:6];    // Bits 7:6
    
    // Instantiate modules
    instruction_memory imem (
        .pc(pc),
        .instr(instruction)
    );
    
    register_file regfile (
        .clk(clk),
        .reset(reset),
        .we(1'b1),          // Always write for simplicity
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
        .opcode(opcode),
        .result(alu_result)
    );
    
    // PC and result logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 0;
        end
        else begin
            pc <= pc + 1;
        end
    end
    
    assign result = alu_result;
endmodule
