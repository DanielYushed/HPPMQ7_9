`include "hppm_params.vh"

module fx_div32 #(
    parameter FBITS = 9
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  signed [31:0] num_in,
    input  signed [31:0] den_in,

    output reg ready,
    output reg signed [15:0] result,
    output reg signed [31:0] result_32
);

    localparam signed [15:0] MAX_FX = 16'sd32767;
    localparam signed [15:0] MIN_FX = -16'sd32768;

    reg [6:0] count;
    reg [95:0] RQ;
    reg [31:0] divisor;
    reg sign_res;

    localparam IDLE = 0, DIVIDE = 1, DONE = 2;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; ready <= 0; result <= 0; result_32 <= 0;
            count <= 0; RQ <= 0; divisor <= 0; sign_res <= 0;
        end else begin
            case (state)
                IDLE: begin
                    ready <= 0;
                    if (start) begin
                        if (den_in == 0) begin
                            result <= (num_in >= 0) ? MAX_FX : MIN_FX;
                            result_32 <= (num_in >= 0) ? 32'sd2147483647 : -32'sd2147483648;
                            ready <= 1;
                        end else begin
                            sign_res <= num_in[31] ^ den_in[31];
                            divisor  <= den_in[31] ? -den_in : den_in;
                            
                            RQ <= { 32'd0, (num_in[31] ? -num_in : num_in), 32'd0 };
                            
                            count <= 32 + FBITS; 
                            state <= DIVIDE;
                        end
                    end
                end

                DIVIDE: begin
                    if (count == 0) begin
                        state <= DONE;
                    end else begin
                        if (RQ[94:63] >= divisor) begin
                            RQ <= { (RQ[94:63] - divisor), RQ[62:0], 1'b1 };
                        end else begin
                            RQ <= { RQ[94:63], RQ[62:0], 1'b0 };
                        end
                        count <= count - 1;
                    end
                end

                DONE: begin
                    result_32 <= sign_res ? -RQ[31:0] : RQ[31:0];

                    if (sign_res) begin
                        if (RQ[31:0] > 32'd32768) result <= MIN_FX;
                        else result <= -RQ[15:0];
                    end else begin
                        if (RQ[31:0] > 32'd32767) result <= MAX_FX;
                        else result <= RQ[15:0];
                    end
                    ready <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule 