// Memoria RAM Simple Dual-Port (Infiere bloques M4K) para almacenar los parámetros de los obstáculos (X, Y, R, K) en Q7.9.

module dual_port_ram (
    input  wire        clk,

    input  wire        we,
    input  wire [9:0]  write_addr,
    input  wire [15:0] data_in,

    input  wire [9:0]  read_addr,
    output reg  [15:0] data_out
);

    reg [15:0] ram [0:1023];

    always @(posedge clk) begin
        if (we) begin
            ram[write_addr] <= data_in;
        end
        data_out <= ram[read_addr];
    end

endmodule 