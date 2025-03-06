module register_file (
    input clk,
    input reset,
    input we,               // Write enable
    input [1:0] rs1_addr,   // Source register 1 address
    input [1:0] rs2_addr,   // Source register 2 address
    input [1:0] rd_addr,    // Destination register address
    input [3:0] rd_data,    // Data to write
    output [3:0] rs1_data,  // Source register 1 data
    output [3:0] rs2_data   // Source register 2 data
);
    reg [3:0] registers [0:3];  // 4 registers, 4-bit each
    
    // Read (combinational)
    assign rs1_data = registers[rs1_addr];
    assign rs2_data = registers[rs2_addr];
    
    // Write (sequential)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            registers[0] <= 4'd2;  // r0 = 2
            registers[1] <= 4'd0;  // r1 = 0
            registers[2] <= 4'd3;  // r2 = 3
            registers[3] <= 4'd0;  // r3 = 0
        end
        else if (we) begin
            registers[rd_addr] <= rd_data;
        end
    end
endmodule
