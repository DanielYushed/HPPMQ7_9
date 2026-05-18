`include "hppm_params.vh"

module jacob_assembler (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  signed [15:0] x, y, L, C1, C2, C3, rad,
    input  signed [31:0] W_acc,
    input  signed [15:0] W_px_acc, W_py_acc,

    input  signed [15:0] sys_m1, sys_m2,       

    input  signed [15:0] term_recta1_in,       
    input  signed [15:0] term_recta2_in,       
    input  signed [15:0] const_f0_base_in,     
    input  signed [15:0] const_f1_base_in,     

    output reg         done,
    output reg  signed [15:0] J00, J01, J02,
    output reg  signed [15:0] J10, J11, J12,
    output reg  signed [15:0] J20, J21, J22,
    output reg  signed [15:0] f0, f1, f2
);

    wire signed [31:0] m1_x_32  = $signed(sys_m1) * $signed(x);
    wire signed [31:0] m2_x_32  = $signed(sys_m2) * $signed(x);

    wire signed [15:0] m1_x  = (m1_x_32 + 32'sd256) >>> 9;
    wire signed [15:0] m2_x  = (m2_x_32 + 32'sd256) >>> 9;

    wire signed [15:0] L_factor = `FX_ONE - L;
    wire signed [31:0] f_mul1_32 = $signed(L_factor) * $signed(term_recta1_in);
    wire signed [31:0] f_mul2_32 = $signed(L_factor) * $signed(term_recta2_in);
    wire signed [15:0] f_mul1 = (f_mul1_32 + 32'sd256) >>> 9;
    wire signed [15:0] f_mul2 = (f_mul2_32 + 32'sd256) >>> 9;
    wire signed [15:0] W_norm = W_acc >>> 9;

    wire signed [15:0] dx_c = x - C1;
    wire signed [15:0] dy_c = y - C2;
    wire signed [15:0] dL_c = L - C3;

    localparam S_IDLE = 1'b0;
    localparam S_CALC = 1'b1;
    reg state;

    reg signed [15:0] dx_c_reg, dy_c_reg, dL_c_reg;
    reg signed [31:0] dx_sq, dy_sq, dL_sq, rad_sq;
    wire signed [63:0] f2_sum;

    assign f2_sum = $signed(dx_sq) + $signed(dy_sq) + $signed(dL_sq) - $signed(rad_sq);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done  <= 1'b0;
            J00 <= 16'd0; J01 <= 16'd0; J02 <= 16'd0;
            J10 <= 16'd0; J11 <= 16'd0; J12 <= 16'd0;
            J20 <= 16'd0; J21 <= 16'd0; J22 <= 16'd0;
            f0  <= 16'd0; f1  <= 16'd0; f2  <= 16'd0;
            dx_c_reg <= 16'd0; dy_c_reg <= 16'd0; dL_c_reg <= 16'd0;
            dx_sq <= 32'd0; dy_sq <= 32'd0; dL_sq <= 32'd0; rad_sq <= 32'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        J00 <= -sys_m1;
                        J01 <= -`FX_ONE;
                        J02 <= term_recta1_in;
                        
                        J10 <= -sys_m2 + W_px_acc;
                        J11 <= -`FX_ONE + W_py_acc;
                        J12 <= term_recta2_in;

                        f0 <= (-y) - m1_x + const_f0_base_in - f_mul1;
                        f1 <= (-y) - m2_x + const_f1_base_in + W_norm - f_mul2;
                        
                        dx_c_reg <= dx_c;
                        dy_c_reg <= dy_c;
                        dL_c_reg <= dL_c;

                        dx_sq  <= $signed(dx_c) * $signed(dx_c);
                        dy_sq  <= $signed(dy_c) * $signed(dy_c);
                        dL_sq  <= $signed(dL_c) * $signed(dL_c);
                        rad_sq <= $signed(rad)  * $signed(rad);

                        state <= S_CALC;
                    end
                end

                S_CALC: begin
                    J20 <= dx_c_reg <<< 1;
                    J21 <= dy_c_reg <<< 1;
                    J22 <= dL_c_reg <<< 1;

                    f2 <= f2_sum[24:9];

                    done  <= 1'b1;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule 