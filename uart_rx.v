
// Formato: 1 bit de inicio, 8 bits de datos, 1 bit de parada, sin paridad.

module uart_rx #(
    parameter CLKS_PER_BIT = 5208
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_pin,
    
    output reg        rx_ready,
    output reg  [7:0] rx_data
);

    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
    localparam DATA  = 3'd2;
    localparam STOP  = 3'd3;
    localparam CLEAN = 3'd4;

    reg [2:0] state;
    reg [15:0] clk_count;
    reg [2:0] bit_index;
    reg [7:0] temp_data;
    
    reg rx_sync1, rx_sync2;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync1 <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync1 <= rx_pin;
            rx_sync2 <= rx_sync1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            clk_count <= 0;
            bit_index <= 0;
            temp_data <= 0;
            rx_ready  <= 0;
            rx_data   <= 0;
        end else begin
            case (state)
                IDLE: begin
                    rx_ready <= 0;
                    clk_count <= 0;
                    bit_index <= 0;
                    if (rx_sync2 == 1'b0) begin
                        state <= START;
                    end
                end
                
                START: begin
                    if (clk_count == (CLKS_PER_BIT / 2)) begin
                        if (rx_sync2 == 1'b0) begin
                            clk_count <= 0;
                            state     <= DATA;
                        end else begin
                            state     <= IDLE;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end
                
                DATA: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count            <= 0;
                        temp_data[bit_index] <= rx_sync2;
                        
                        if (bit_index == 7) begin
                            state <= STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end
                
                STOP: begin
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        rx_data  <= temp_data;
                        rx_ready <= 1'b1;
                        state    <= CLEAN;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end
                
                CLEAN: begin
                    rx_ready <= 1'b0;
                    state    <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule