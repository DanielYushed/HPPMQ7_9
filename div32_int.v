module div32_int (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  signed [31:0] num_in,
    input  signed [31:0] den_in,
    
    output reg         ready,
    output reg  signed [31:0] result
);

    reg [5:0]  count;
    reg [63:0] P_A;
    reg [31:0] divisor;
    reg        sign_res;
    
    localparam IDLE = 0, DIVIDE = 1, DONE = 2;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            ready <= 0;
            result <= 0;
            count <= 0;
            P_A <= 0;
            divisor <= 0;
            sign_res <= 0;
        end else begin
            case (state)
                IDLE: begin
                    ready <= 0;
                    if (start) begin
                        if (den_in == 0) begin
                            result <= (num_in >= 0) ? 32'h7FFFFFFF : 32'h80000000;
                            ready  <= 1;
                        end else begin
                            sign_res <= num_in[31] ^ den_in[31];
                            divisor  <= den_in[31] ? -den_in : den_in;
                            P_A <= {32'd0, (num_in[31] ? -num_in : num_in)};
                            
                            count <= 32;
                            state <= DIVIDE;
                        end
                    end
                end
                
                DIVIDE: begin
                    if (count == 0) begin
                        state <= DONE;
                    end else begin
                        if (P_A[62:31] >= divisor) begin
                            P_A <= { (P_A[62:31] - divisor), P_A[30:0], 1'b1 };
                        end else begin
                            P_A <= { P_A[62:0], 1'b0 };
                        end
                        count <= count - 1;
                    end
                end
                
                DONE: begin
                    result <= sign_res ? -P_A[31:0] : P_A[31:0];
                    ready  <= 1;
                    state  <= IDLE;
                end
            endcase
        end
    end

endmodule 