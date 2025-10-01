module instruction_memory (
    input [3:0] pc,
    output reg [11:0] instr
);
    reg [11:0] mem [0:15];
    
    initial begin
        mem[0] = 12'b000100100000;
        mem[1] = 12'b001000010001;
        mem[2] = 12'b001100100000;
        mem[3] = 12'b000000000000;
        mem[4] = 12'b000000000000;
        mem[5] = 12'b000000000000;
        mem[6] = 12'b000000000000;
        mem[7] = 12'b000000000000;
        mem[8] = 12'b000000000000;
        mem[9] = 12'b000000000000;
        mem[10] = 12'b000000000000;
        mem[11] = 12'b000000000000;
        mem[12] = 12'b000000000000;
        mem[13] = 12'b000000000000;
        mem[14] = 12'b000000000000;
        mem[15] = 12'b000000000000;
    end
    
    always @(*) begin
        instr = mem[pc];
    end
endmodule
