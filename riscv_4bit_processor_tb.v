module riscv_4bit_processor_tb;
    // Signals
    reg clk;
    reg reset;
    wire [3:0] result;
    
    // Instantiate processor
    riscv_4bit_processor dut (
        .clk(clk),
        .reset(reset),
        .result(result)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns period
    end
    
    // Test sequence
    initial begin
        $dumpfile("processor.vcd");
        $dumpvars(0, riscv_4bit_processor_tb);
        
        // Reset
        reset = 1;
        #10;
        reset = 0;
        
        // Run for 4 instructions (40ns)
        #40;
        
        // Display results
        $display("Final Result: %d", result);
        
        // Check expected result
        if (result == 4'd0)  // Final ADD: 3 + 5 = 8, truncated to 0 in 4-bit
            $display("Test Passed!");
        else
            $display("Test Failed! Expected 0, got %d", result);
            
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("Time=%0t: PC=%d, Result=%d", $time, dut.pc, result);
    end
endmodule
