module riscv_4bit_processor_tb;
    reg clk;
    reg reset;
    wire [3:0] result;
    
    riscv_4bit_processor dut (
        .clk(clk),
        .reset(reset),
        .result(result)
    );
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    initial begin
        $dumpfile("processor.vcd");
        $dumpvars(0, riscv_4bit_processor_tb);
        
        reset = 1;
        #10;
        reset = 0;
        
        #40;
        
        #1;
        $display("Final Result: %d", result);
        
        if (result === 4'd0)
            $display("Test Passed!");
        else
            $display("Test Failed! Expected 0, got %d", result);
            
        $finish;
    end
    
    initial begin
        $monitor("Time = %0t: PC = %d, Result = %d", $time, dut.pc, result);
    end
endmodule
