module program_counter (
    input clk,
    input reset,
    output reg [3:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 4'b0000;
        end
        else begin
            pc <= pc + 1;
        end
    end
endmodule
