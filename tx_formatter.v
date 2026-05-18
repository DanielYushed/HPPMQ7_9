module tx_formatter (
    input  wire clk,
    input  wire rst_n,
    input  wire start,

    input  signed [15:0] x_in,
    input  signed [15:0] y_in,
    input  signed [15:0] L_in,

    input  wire        tx_done,
    output reg         tx_start,
    output reg  [7:0]  tx_data,

    output reg done
);

    reg [15:0] x_reg;
    reg [15:0] y_reg;
    reg [15:0] L_reg;

    localparam S_IDLE      = 3'd0;
    localparam S_LOAD_BYTE = 3'd1;
    localparam S_WAIT_TX   = 3'd2;
    localparam S_DONE      = 3'd3;

    reg [2:0] state;
    reg [2:0] byte_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            tx_start <= 0;
            tx_data  <= 0;
            done     <= 0;
            byte_cnt <= 0;
            x_reg    <= 0;
            y_reg    <= 0;
            L_reg    <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    tx_start <= 0;
                    if (start) begin
                        x_reg <= x_in;
                        y_reg <= y_in;
                        L_reg <= L_in;
                        byte_cnt <= 0;
                        state <= S_LOAD_BYTE;
                    end
                end

                S_LOAD_BYTE: begin
                    case (byte_cnt)
                        3'd0: tx_data <= x_reg[15:8]; // X (MSB)
                        3'd1: tx_data <= x_reg[7:0];  // X (LSB)
                        3'd2: tx_data <= y_reg[15:8]; // Y (MSB)
                        3'd3: tx_data <= y_reg[7:0];  // Y (LSB)
                        3'd4: tx_data <= L_reg[15:8]; // L (MSB)
                        3'd5: tx_data <= L_reg[7:0];  // L (LSB)
                        default: tx_data <= 8'h00;
                    endcase
                    
                    tx_start <= 1'b1;
                    state <= S_WAIT_TX;
                end

                S_WAIT_TX: begin
                    tx_start <= 1'b0;
                    
                    if (tx_done) begin
                        if (byte_cnt == 3'd5) begin
                            state <= S_DONE;
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                            state <= S_LOAD_BYTE;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule 