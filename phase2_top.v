// Módulo para UART Desempaquetador y RAM 
`include "hppm_params.vh"

module phase2_top (
    input  wire clk,
    input  wire rst_n,
    input  wire rx_pin,

    output wire [15:0] sys_nc, sys_rr, sys_m1, sys_m2, sys_maxsteps,
    output wire [15:0] sys_a0, sys_b0, sys_a1, sys_b1,

    output wire config_done,

    input  wire [9:0]  ram_read_addr,
    output wire [15:0] ram_read_data
);

    wire        rx_ready_net;
    wire [7:0]  rx_data_net;
    
    wire        ram_we_net;
    wire [9:0]  ram_write_addr_net;
    wire [15:0] ram_data_in_net;

    uart_rx #(
        .CLKS_PER_BIT(`CLKS_PER_BIT) 
    ) u_uart (
        .clk(clk),
        .rst_n(rst_n),
        .rx_pin(rx_pin),
        .rx_ready(rx_ready_net),
        .rx_data(rx_data_net)
    );

    data_unpacker u_unpacker (
        .clk(clk),
        .rst_n(rst_n),
        .rx_ready(rx_ready_net),
        .rx_data(rx_data_net),
        
        .sys_nc(sys_nc), .sys_rr(sys_rr), .sys_m1(sys_m1), .sys_m2(sys_m2),
        .sys_maxsteps(sys_maxsteps), .sys_a0(sys_a0), .sys_b0(sys_b0),
        .sys_a1(sys_a1), .sys_b1(sys_b1),
        .ram_we(ram_we_net),
        .ram_addr(ram_write_addr_net),
        .ram_data_out(ram_data_in_net),
        .config_done(config_done)
    );

    dual_port_ram u_ram (
        .clk(clk),
        // Escritura
        .we(ram_we_net),
        .write_addr(ram_write_addr_net),
        .data_in(ram_data_in_net),
        
        // Lectura
        .read_addr(ram_read_addr),
        .data_out(ram_read_data)
    );

endmodule 