module control_unit (
    input [11:0] instruction,  // 12-bit instruction
    output reg alu_op,         // ALU operation (0 = ADD, 1 = SUB)
    output reg reg_we          // Register file write enable
);
    wire opcode;
    
    // Extract opcode from instruction (bit 0)
    assign opcode = instruction[0];
    
    // Control logic
    always @(*) begin
        // Default values
        alu_op = 1'b0;
        reg_we = 1'b0;
        
        // Decode instruction
        case (opcode)
            1'b0: begin  // ADD
                alu_op = 1'b0;  // ALU performs addition
                reg_we = 1'b1;  // Enable register write
            end
            1'b1: begin  // SUB
                alu_op = 1'b1;  // ALU performs subtraction
                reg_we = 1'b1;  // Enable register write
            end
            default: begin
                alu_op = 1'b0;  // Default to ADD (safe operation)
                reg_we = 1'b0;  // Disable register write (NOP)
            end
        endcase
    end
endmodule
