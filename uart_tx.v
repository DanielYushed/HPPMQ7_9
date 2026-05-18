module uart_tx #(
    parameter CLKS_PER_BIT = 5208
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tx_start,
    input  wire [7:0]  tx_data,

    output reg         tx_active,
    output reg         tx_pin,
    output reg         tx_done
);

    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_count;
    reg [2:0]  bit_index;
    reg [7:0]  data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            tx_pin    <= 1'b1; 
            tx_active <= 1'b0;
            tx_done   <= 1'b0;
            clk_count <= 0;
            bit_index <= 0;
            data_reg  <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx_pin    <= 1'b1;
                    tx_done   <= 1'b0;
                    clk_count <= 0;
                    bit_index <= 0;

                    if (tx_start) begin
                        tx_active <= 1'b1;
                        data_reg  <= tx_data;
                        state     <= START;
                    end else begin
                        tx_active <= 1'b0;
                    end
                end

                START: begin
                    tx_pin <= 1'b0;
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
                        state     <= DATA;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                DATA: begin
                    tx_pin <= data_reg[bit_index]; 
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        clk_count <= 0;
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
                    tx_pin <= 1'b1;
                    if (clk_count == CLKS_PER_BIT - 1) begin
                        tx_done   <= 1'b1; 
                        state     <= IDLE;
                        clk_count <= 0;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end
            endcase
        end
    end
endmodule 