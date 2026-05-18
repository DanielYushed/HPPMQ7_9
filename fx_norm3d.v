// Módulo: Norma 3D y Raíz Cuadrada Entera (fx_norm3d)
// Calcula sqrt(x^2 + y^2 + z^2).

module fx_norm3d (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    
    input  signed [15:0] x,
    input  signed [15:0] y,
    input  signed [15:0] z,
    
    output reg         done,
    output reg  signed [15:0] norm_out
);

    reg signed [15:0] x_reg, y_reg, z_reg;
    reg signed [31:0] x2_reg, y2_reg, z2_reg;
    reg signed [31:0] sum_sq_reg;
    reg [31:0] val;
    reg [31:0] res;
    reg [31:0] bit_mask;

    localparam S_IDLE     = 3'd0;
    localparam S_SQUARING = 3'd1;
    localparam S_SUMMING  = 3'd2;
    localparam S_PRE_CALC = 3'd3;
    localparam S_CALC     = 3'd4;
    localparam S_DONE     = 3'd5;
    
    reg [2:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0; norm_out <= 0;
            val <= 0; res <= 0; bit_mask <= 0;
            
            x_reg <= 0; y_reg <= 0; z_reg <= 0;
            x2_reg <= 0; y2_reg <= 0; z2_reg <= 0;
            sum_sq_reg <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        x_reg <= x;
                        y_reg <= y;
                        z_reg <= z;
                        state <= S_SQUARING;
                    end
                end

                S_SQUARING: begin
                    x2_reg <= x_reg * x_reg;
                    y2_reg <= y_reg * y_reg;
                    z2_reg <= z_reg * z_reg;
                    state <= S_SUMMING;
                end

                S_SUMMING: begin
                    sum_sq_reg <= x2_reg + y2_reg + z2_reg;
                    state <= S_PRE_CALC;
                end

                S_PRE_CALC: begin
                    if (sum_sq_reg <= 0) begin 
                        norm_out <= 0;
                        done <= 1;
                        state <= S_DONE;
                    end else begin
                        val <= sum_sq_reg;
                        res <= 0;
                        bit_mask <= 32'h40000000; // 1 << 30
                        state <= S_CALC;
                    end
                end

                S_CALC: begin
                    if (bit_mask != 0) begin
                        if (val >= res + bit_mask) begin
                            val <= val - (res + bit_mask);
                            res <= (res >> 1) + bit_mask;
                        end else begin
                            res <= res >> 1;
                        end
                        bit_mask <= bit_mask >> 2;
                    end else begin
                        norm_out <= res[15:0];
                        done <= 1;
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    done <= 0;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule 