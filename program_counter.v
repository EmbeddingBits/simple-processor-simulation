module program_counter (
    input clk,              // Clock signal
    input reset,            // Reset signal (active high)
    output reg [3:0] pc     // 4-bit program counter
);
    // PC logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 4'b0000;  // Reset PC to 0
        end
        else begin
            pc <= pc + 1;   // Increment PC by 1
        end
    end
endmodule
