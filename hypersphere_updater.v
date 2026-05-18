`include "hppm_params.vh"

module hypersphere_updater (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  signed [15:0] J00, J01, J02,
    input  signed [15:0] J10, J11, J12,

    input  signed [15:0] xa, ya, La,
    input  signed [15:0] x_old, y_old, L_old,

    input  signed [15:0] sys_rr,
    input  signed [15:0] r_prev,
    input  signed [15:0] m_L1_in,
    input  signed [15:0] signo,

    output reg         done,
    output reg  signed [15:0] C1_out, C2_out, C3_out,
    output reg  signed [15:0] Dd0_out, Dd1_out, Dd2_out,
    output reg  signed [15:0] rad_out,
    output reg  signed [15:0] m_L1_out,
    output reg  signed [15:0] norxa_out,
    output reg  signed [15:0] norya_out,

    output reg         norm_start_out,
    input  wire        norm_done_in,
    output reg  signed [15:0] norm_x_out, norm_y_out, norm_z_out,
    input  wire signed [15:0] norm_res_in
);

    wire signed [31:0] det32_raw = -((J00 * J11) - (J01 * J10)) >>> 9;
    wire signed [31:0] ad1_32_raw = ((J02 * J11) - (J12 * J01)) >>> 9;
    wire signed [31:0] ad2_32_raw = ((J12 * J00) - (J02 * J10)) >>> 9;
    
    wire signed [15:0] det16 = det32_raw[15:0];
    wire signed [15:0] ad1_16 = ad1_32_raw[15:0];
    wire signed [15:0] ad2_16 = ad2_32_raw[15:0];

    reg start_div_g1;
    wire ready_exp;
    wire signed [15:0] res_exp;
    reg f_exp, f_norm1;

    reg start_div_g2;
    wire ready_nx, ready_ny, ready_nL;
    wire signed [15:0] res_nx, res_ny, res_nL;
    reg f_nx, f_ny, f_nL;

    reg start_div_g3;
    wire ready_fact;
    wire signed [15:0] res_fact;

    reg signed [15:0] res_exp_reg;   
    reg signed [15:0] norm_res_reg1; 
    reg signed [15:0] rad_calc_reg;  

    reg signed [15:0] exp_out_reg;

    wire signed [15:0] exp_out;
    fx_exp_neg u_exp (.x(res_exp_reg), .y(exp_out)); 

    function [15:0] abs_val;
        input [15:0] val;
        begin abs_val = (val[15]) ? -val : val; end
    endfunction

    wire signed [15:0] abs_mL1 = abs_val(m_L1_in);
    wire signed [15:0] abs_det = abs_val(det16);
    wire signed [15:0] div_exp = (abs_det < `FX_TINY) ? `FX_TINY : abs_det;
    
    fx_div32 u_div_exp (
        .clk(clk), .rst_n(rst_n), .start(start_div_g1),
        .num_in({16'd0, abs_mL1}), .den_in({{16{div_exp[15]}}, div_exp}), 
        .ready(ready_exp), .result(res_exp)
    );

    wire signed [15:0] res_r2 = r_prev >>> 1; 
    wire signed [31:0] r_prev_mul = r_prev * 32'sd21845;
    wire signed [15:0] res_r3 = r_prev_mul >>> 16;

    reg signed [15:0] nBA;
    
    fx_div32 u_div_nx (
        .clk(clk), .rst_n(rst_n), .start(start_div_g2),
        .num_in(ad1_32_raw), .den_in({{16{nBA[15]}}, nBA}), 
        .ready(ready_nx), .result(res_nx)
    );

    fx_div32 u_div_ny (
        .clk(clk), .rst_n(rst_n), .start(start_div_g2),
        .num_in(ad2_32_raw), .den_in({{16{nBA[15]}}, nBA}), 
        .ready(ready_ny), .result(res_ny)
    );

    fx_div32 u_div_nL (
        .clk(clk), .rst_n(rst_n), .start(start_div_g2),
        .num_in(det32_raw), .den_in({{16{nBA[15]}}, nBA}), 
        .ready(ready_nL), .result(res_nL)
    );

    reg signed [15:0] norxa, norya, norLa;
    reg signed [15:0] norBA_corr;

    reg signed [15:0] next_rad_reg;
    
    wire signed [31:0] rad_mul = sys_rr * (`FX_ONE + exp_out_reg);

    reg signed [31:0] mul_rad_nx_reg, mul_rad_ny_reg, mul_rad_nL_reg;
    reg signed [31:0] mul_sig_nx_reg, mul_sig_ny_reg, mul_sig_nL_reg;
    reg signed [15:0] next_C1_reg, next_C2_reg, next_C3_reg;

    wire signed [15:0] factor_den = (norBA_corr < `FX_TINY) ? `FX_TINY : norBA_corr;
    wire signed [15:0] diff_rad = next_rad_reg - r_prev; 
    
    fx_div32 u_div_factor (
        .clk(clk), .rst_n(rst_n), .start(start_div_g3),
        .num_in({{16{diff_rad[15]}}, diff_rad}), .den_in({{16{factor_den[15]}}, factor_den}), 
        .ready(ready_fact), .result(res_fact)
    );

    localparam S_IDLE          = 4'd0;
    localparam S_START_G1_NBA  = 4'd1;
    localparam S_WAIT_G1_NBA   = 4'd2;
    localparam S_PIPE_EXP1     = 4'd14;
    localparam S_PIPE_EXP2     = 4'd15; 
    localparam S_PIPE_RAD      = 4'd3; 
    localparam S_START_G2      = 4'd4;
    localparam S_WAIT_G2       = 4'd5;
    localparam S_PIPE_MUL_RAD  = 4'd6;  
    localparam S_START_CORR    = 4'd7;
    localparam S_WAIT_CORR     = 4'd8;
    localparam S_PIPE_SIG      = 4'd9;  
    localparam S_START_FACT    = 4'd10;
    localparam S_WAIT_FACT     = 4'd11;
    localparam S_PIPE_C        = 4'd12; 
    localparam S_CALC_FINAL    = 4'd13;

    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0; 
            norm_start_out <= 0; start_div_g1 <= 0; start_div_g2 <= 0; start_div_g3 <= 0;
            f_exp <= 0; f_norm1 <= 0; f_nx <= 0; f_ny <= 0; f_nL <= 0;
            C1_out <= 0; C2_out <= 0; C3_out <= 0;
            Dd0_out <= 0; Dd1_out <= 0; Dd2_out <= 0; rad_out <= 0;
            norxa_out <= 0; norya_out <= 0;
            
            res_exp_reg <= 0; norm_res_reg1 <= 0; rad_calc_reg <= 0; exp_out_reg <= 0;
            next_rad_reg <= 0;
            mul_rad_nx_reg <= 0; mul_rad_ny_reg <= 0; mul_rad_nL_reg <= 0;
            mul_sig_nx_reg <= 0; mul_sig_ny_reg <= 0; mul_sig_nL_reg <= 0;
            next_C1_reg <= 0; next_C2_reg <= 0; next_C3_reg <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; f_exp <= 0; f_norm1 <= 0; f_nx <= 0; f_ny <= 0; f_nL <= 0;
                    if (start) begin
                        m_L1_out <= det16; 
                        norm_x_out <= ad1_16; norm_y_out <= ad2_16; norm_z_out <= det16;
                        state <= S_START_G1_NBA;
                    end
                end

                S_START_G1_NBA: begin norm_start_out <= 1; start_div_g1 <= 1; state <= S_WAIT_G1_NBA; end

                S_WAIT_G1_NBA: begin
                    norm_start_out <= 0; start_div_g1 <= 0;
                    
                    if (ready_exp) begin f_exp <= 1; res_exp_reg <= res_exp; end
                    if (norm_done_in) begin f_norm1 <= 1; norm_res_reg1 <= norm_res_in; end

                    if ((f_exp || ready_exp) && (f_norm1 || norm_done_in)) begin
                        if (norm_done_in) nBA <= (norm_res_in < `FX_TINY) ? `FX_TINY : norm_res_in;
                        else nBA <= (norm_res_reg1 < `FX_TINY) ? `FX_TINY : norm_res_reg1;
                        
                        if (ready_exp) res_exp_reg <= res_exp;

                        state <= S_PIPE_EXP1;
                    end
                end

                S_PIPE_EXP1: begin
                    exp_out_reg <= exp_out;
                    f_exp <= 0; f_norm1 <= 0;
                    state <= S_PIPE_EXP2;
                end

                S_PIPE_EXP2: begin
                    rad_calc_reg <= (rad_mul + `FX_HALF) >>> 9;
                    state <= S_PIPE_RAD;
                end

                S_PIPE_RAD: begin
                    next_rad_reg <= ((r_prev - rad_calc_reg) > res_r3) ? (res_r2 + sys_rr) : rad_calc_reg;
                    state <= S_START_G2;
                end

                S_START_G2: begin start_div_g2 <= 1; state <= S_WAIT_G2; end

                S_WAIT_G2: begin
                    start_div_g2 <= 0;
                    if (ready_nx) f_nx <= 1; if (ready_ny) f_ny <= 1; if (ready_nL) f_nL <= 1;

                    if ((f_nx || ready_nx) && (f_ny || ready_ny) && (f_nL || ready_nL)) begin
                        norxa <= res_nx; norya <= res_ny; norLa <= res_nL;
                        norm_x_out <= xa - x_old; norm_y_out <= ya - y_old; norm_z_out <= La - L_old;
                        f_nx <= 0; f_ny <= 0; f_nL <= 0;
                        state <= S_PIPE_MUL_RAD; 
                    end
                end

                S_PIPE_MUL_RAD: begin
                    mul_rad_nx_reg <= next_rad_reg * norxa;
                    mul_rad_ny_reg <= next_rad_reg * norya;
                    mul_rad_nL_reg <= next_rad_reg * norLa;
                    state <= S_START_CORR;
                end

                S_START_CORR: begin norm_start_out <= 1; state <= S_WAIT_CORR; end

                S_WAIT_CORR: begin
                    norm_start_out <= 0;
                    if (norm_done_in) begin
                        norBA_corr <= (norm_res_in < `FX_TINY) ? `FX_TINY : norm_res_in;
                        state <= S_PIPE_SIG; 
                    end
                end

                S_PIPE_SIG: begin
                    mul_sig_nx_reg <= signo * ((mul_rad_nx_reg + `FX_HALF) >>> 9);
                    mul_sig_ny_reg <= signo * ((mul_rad_ny_reg + `FX_HALF) >>> 9);
                    mul_sig_nL_reg <= signo * ((mul_rad_nL_reg + `FX_HALF) >>> 9);
                    state <= S_START_FACT;
                end

                S_START_FACT: begin start_div_g3 <= 1; state <= S_WAIT_FACT; end
                
                S_WAIT_FACT: begin 
                    start_div_g3 <= 0; 
                    if (ready_fact) begin
                        state <= S_PIPE_C; 
                    end 
                end

                S_PIPE_C: begin
                    next_C1_reg <= xa + `FX_MUL_ROUND(res_fact, xa - x_old);
                    next_C2_reg <= ya + `FX_MUL_ROUND(res_fact, ya - y_old);
                    next_C3_reg <= La + `FX_MUL_ROUND(res_fact, La - L_old);
                    state <= S_CALC_FINAL;
                end

                S_CALC_FINAL: begin
                    rad_out <= next_rad_reg; 
                    C1_out <= next_C1_reg; C2_out <= next_C2_reg; C3_out <= next_C3_reg;
                    Dd0_out <= next_C1_reg + ((mul_sig_nx_reg + `FX_HALF) >>> 9); 
                    Dd1_out <= next_C2_reg + ((mul_sig_ny_reg + `FX_HALF) >>> 9); 
                    Dd2_out <= next_C3_reg + ((mul_sig_nL_reg + `FX_HALF) >>> 9); 
                    norxa_out <= norxa;
                    norya_out <= norya;
                    done <= 1; state <= S_IDLE;
                end 
                
                default: begin
                    state <= S_IDLE; done <= 0; norm_start_out <= 0; 
                    start_div_g1 <= 0; start_div_g2 <= 0; start_div_g3 <= 0;
                end
            endcase
        end
    end
endmodule 