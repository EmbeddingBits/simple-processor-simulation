module control_unit (
    input [11:0] instruction,
    output reg alu_op,
    output reg reg_we
);
    wire opcode;
    
    assign opcode = instruction[0];
    
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
