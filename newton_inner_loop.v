`include "hppm_params.vh"

module newton_inner_loop (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  signed [15:0] x_in, y_in, L_in,
    input  signed [15:0] C1_in, C2_in, C3_in,
    input  signed [15:0] rad_in, W_0_in, Q_in,

    input  signed [15:0] sys_nc, sys_m1, sys_m2, sys_a0, sys_b0, sys_a1, sys_b1,
    input  signed [15:0] sys_paro,

    output reg         rep_start_out,
    input  wire        rep_done_in,
    output wire signed [15:0] rep_robot_x_out,
    output wire signed [15:0] rep_robot_y_out,
    input  wire signed [31:0] rep_W_acc_in,
    input  wire signed [15:0] rep_W_px_in,
    input  wire signed [15:0] rep_W_py_in,

    output reg         done,
    output reg  signed [15:0] x_out, y_out, L_out,
    output reg  [15:0] iters_out,

    output wire signed [15:0] J00_out, J01_out, J02_out,
    output wire signed [15:0] J10_out, J11_out, J12_out,
    output wire signed [15:0] J20_out, J21_out, J22_out,

    output reg         norm_start_out,
    input  wire        norm_done_in,
    output wire signed [15:0] norm_x_out, norm_y_out, norm_z_out,
    input  wire signed [15:0] norm_res_in
);

    reg signed [15:0] x_reg, y_reg, L_reg;
    reg signed [15:0] err_reg;
    reg [15:0] i_cnt;

    reg signed [15:0] term_recta1_reg;
    reg signed [15:0] term_recta2_reg;
    reg signed [15:0] const_f0_base_reg;
    reg signed [15:0] const_f1_base_reg;
    reg signed [15:0] sys_m1_reg;
    reg signed [15:0] sys_m2_reg;

    assign rep_robot_x_out = x_reg;
    assign rep_robot_y_out = y_reg;

    wire signed [31:0] m1_a0_32 = $signed(sys_m1) * $signed(sys_a0);
    wire signed [31:0] m1_a1_32 = $signed(sys_m1) * $signed(sys_a1);
    wire signed [31:0] m2_a0_32 = $signed(sys_m2) * $signed(sys_a0);
    wire signed [31:0] m2_a1_32 = $signed(sys_m2) * $signed(sys_a1);

    wire signed [15:0] m1_a0 = (m1_a0_32 + `FX_HALF) >>> 9;
    wire signed [15:0] m1_a1 = (m1_a1_32 + `FX_HALF) >>> 9;
    wire signed [15:0] m2_a0 = (m2_a0_32 + `FX_HALF) >>> 9;
    wire signed [15:0] m2_a1 = (m2_a1_32 + `FX_HALF) >>> 9;

    wire signed [15:0] term_recta1_pre = (-sys_b0) - m1_a0 + m1_a1 + sys_b1;
    wire signed [15:0] term_recta2_pre = (-sys_b0) - m2_a0 + sys_b1 + m2_a1 + W_0_in - Q_in;
    wire signed [15:0] const_f0_base_pre = m1_a1 + sys_b1;
    wire signed [15:0] const_f1_base_pre = sys_b1 + m2_a1 - Q_in;

    reg jac_start; wire jac_done;
    wire signed [15:0] f0, f1, f2;
    
    jacob_assembler u_jac (
        .clk(clk), .rst_n(rst_n), .start(jac_start),
        .x(x_reg), .y(y_reg), .L(L_reg), .C1(C1_in), .C2(C2_in), .C3(C3_in),
        .rad(rad_in),
        .W_acc(rep_W_acc_in), .W_px_acc(rep_W_px_in), .W_py_acc(rep_W_py_in),
        .sys_m1(sys_m1_reg), .sys_m2(sys_m2_reg),
        .term_recta1_in(term_recta1_reg), .term_recta2_in(term_recta2_reg),
        .const_f0_base_in(const_f0_base_reg), .const_f1_base_in(const_f1_base_reg),
        .done(jac_done),
        .J00(J00_out), .J01(J01_out), .J02(J02_out),
        .J10(J10_out), .J11(J11_out), .J12(J12_out),
        .J20(J20_out), .J21(J21_out), .J22(J22_out),
        .f0(f0), .f1(f1), .f2(f2)
    );

    reg newt_start; wire newt_done;
    wire signed [15:0] dx, dy, dL;
    newton_step u_newt (
        .clk(clk), .rst_n(rst_n), .start(newt_start),
        .J00(J00_out), .J01(J01_out), .J02(J02_out),
        .J10(J10_out), .J11(J11_out), .J12(J12_out),
        .J20(J20_out), .J21(J21_out), .J22(J22_out),
        .f0(f0), .f1(f1), .f2(f2),
        .done(newt_done), .delta_x(dx), .delta_y(dy), .delta_L(dL)
    );

    assign norm_x_out = f0; 
    assign norm_y_out = f1; 
    assign norm_z_out = f2;

    localparam S_IDLE       = 4'd0, S_CHECK_COND = 4'd1, S_EVAL_JAC_1 = 4'd2, S_WAIT_REP_1 = 4'd3,
               S_WAIT_JAC_1 = 4'd4, S_EVAL_NEWT  = 4'd5, S_WAIT_NEWT  = 4'd6, S_UPDATE_XY  = 4'd7,
               S_CHECK_I    = 4'd8, S_EVAL_JAC_2 = 4'd9, S_WAIT_REP_2 = 4'd10, S_WAIT_JAC_2 = 4'd11,
               S_EVAL_NORM  = 4'd12, S_WAIT_NORM  = 4'd13, S_END_LOOP   = 4'd14;

    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0;
            rep_start_out <= 0; jac_start <= 0; newt_start <= 0; norm_start_out <= 0;
            x_reg <= 0; y_reg <= 0; L_reg <= 0; err_reg <= 0; i_cnt <= 0;
            term_recta1_reg <= 0; term_recta2_reg <= 0; const_f0_base_reg <= 0; const_f1_base_reg <= 0;
            sys_m1_reg <= 0; sys_m2_reg <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        x_reg <= x_in; y_reg <= y_in; L_reg <= L_in; err_reg <= `FX_ONE; i_cnt <= 0;
                        term_recta1_reg <= term_recta1_pre;
                        term_recta2_reg <= term_recta2_pre;
                        const_f0_base_reg <= const_f0_base_pre;
                        const_f1_base_reg <= const_f1_base_pre;
                        sys_m1_reg <= sys_m1;
                        sys_m2_reg <= sys_m2;

                        state <= S_CHECK_COND;
                    end
                end

                S_CHECK_COND: state <= ((err_reg > sys_paro) && (i_cnt < `NEWTON_MAX_ITERS)) ? S_EVAL_JAC_1 : S_END_LOOP;

                S_EVAL_JAC_1: begin rep_start_out <= 1; state <= S_WAIT_REP_1; end
                S_WAIT_REP_1: begin rep_start_out <= 0; if (rep_done_in) begin jac_start <= 1; state <= S_WAIT_JAC_1; end end
                S_WAIT_JAC_1: begin jac_start <= 0; if (jac_done) state <= S_EVAL_NEWT; end

                S_EVAL_NEWT:  begin newt_start <= 1; state <= S_WAIT_NEWT; end
                S_WAIT_NEWT:  begin newt_start <= 0; if (newt_done) state <= S_UPDATE_XY; end

                S_UPDATE_XY: begin x_reg <= x_reg - dx; y_reg <= y_reg - dy; L_reg <= L_reg - dL; state <= S_CHECK_I; end

                S_CHECK_I: begin
                    state <= S_EVAL_JAC_2; 
                end

                S_EVAL_JAC_2: begin rep_start_out <= 1; state <= S_WAIT_REP_2; end
                S_WAIT_REP_2: begin rep_start_out <= 0; if (rep_done_in) begin jac_start <= 1; state <= S_WAIT_JAC_2; end end
                S_WAIT_JAC_2: begin jac_start <= 0; if (jac_done) state <= S_EVAL_NORM; end

                S_EVAL_NORM:  begin norm_start_out <= 1; state <= S_WAIT_NORM; end
                S_WAIT_NORM: begin
                    norm_start_out <= 0;
                    if (norm_done_in) begin err_reg <= norm_res_in; i_cnt <= i_cnt + 1'b1; state <= S_CHECK_COND; end
                end

                S_END_LOOP: begin
                    x_out <= x_reg; y_out <= y_reg; L_out <= L_reg; iters_out <= i_cnt; done <= 1; state <= S_IDLE;
                end
                
                default: begin state <= S_IDLE; done <= 0; rep_start_out <= 0; jac_start <= 0; newt_start <= 0; norm_start_out <= 0; end
            endcase
        end
    end
endmodule