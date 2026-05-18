module hppm_top (
    input  wire clk,
    input  wire rst_n,
    input  wire rx_pin,
    output wire tx_pin,
    output wire hppm_finished,
    
    output wire debug_config_done,
    output wire [4:0] debug_state,
    output wire debug_rx_line,
    output wire debug_tx_line
);

    wire [15:0] sys_nc, sys_rr, sys_m1, sys_m2, sys_maxsteps;
    wire [15:0] sys_a0, sys_b0, sys_a1, sys_b1;
    
    wire config_done;
    wire [9:0]  ram_read_addr;
    wire [15:0] ram_read_data;

    assign debug_config_done = config_done;
    assign debug_rx_line = rx_pin; 
    assign debug_tx_line = tx_pin; 
    
    phase2_top u_fase2 (
        .clk(clk), .rst_n(rst_n), .rx_pin(rx_pin),
        .sys_nc(sys_nc), .sys_rr(sys_rr), .sys_m1(sys_m1), .sys_m2(sys_m2),
        .sys_maxsteps(sys_maxsteps), .sys_a0(sys_a0), .sys_b0(sys_b0),
        .sys_a1(sys_a1), .sys_b1(sys_b1),
        
        .config_done(config_done),
        .ram_read_addr(ram_read_addr), .ram_read_data(ram_read_data)
    );

    hppm_core u_cerebro (
        .clk(clk), .rst_n(rst_n), .config_done(config_done),
        .sys_nc(sys_nc), .sys_rr(sys_rr), .sys_m1(sys_m1), .sys_m2(sys_m2),
        .sys_maxsteps(sys_maxsteps), .sys_a0(sys_a0), .sys_b0(sys_b0),
        .sys_a1(sys_a1), .sys_b1(sys_b1),
        .ram_read_addr(ram_read_addr), .ram_read_data(ram_read_data),
        .tx_pin(tx_pin), .hppm_finished(hppm_finished),
        .current_state(debug_state)
    );

endmodule 