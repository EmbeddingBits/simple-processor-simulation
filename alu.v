module alu (
    input [3:0] a,          // First operand
    input [3:0] b,          // Second operand
    input opcode,           // 0 = ADD, 1 = SUB
    output reg [3:0] result // ALU result
);
    always @(*) begin
        case (opcode)
            1'b0: result = a + b;  // ADD, truncated to 4 bits
            1'b1: result = a - b;  // SUB, truncated to 4 bits
            default: result = 4'b0;
        endcase
    end
endmodule
