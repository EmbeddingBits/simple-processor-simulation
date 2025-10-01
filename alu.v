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
