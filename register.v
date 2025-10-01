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
