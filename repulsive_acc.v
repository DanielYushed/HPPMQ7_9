`include "hppm_params.vh"

module repulsive_acc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  signed [15:0] robot_x,
    input  signed [15:0] robot_y,
    input  wire   [15:0] sys_nc,

    output reg  [9:0]  ram_read_addr,
    input  wire [15:0] ram_read_data,

    output reg         done,
    output reg  signed [31:0] W_acc_out,
    output reg  signed [15:0] W_px_out,
    output reg  signed [15:0] W_py_out
);

    reg [15:0] k_count;
    reg signed [31:0] W_acc;
    reg signed [31:0] W_px_acc, W_py_acc;
    reg signed [15:0] obs_x, obs_y, obs_r, obs_k;
    reg signed [15:0] dx, dy;
    
    reg signed [15:0] term_reg;
    reg signed [31:0] term2_reg;
    reg signed [31:0] aux_Wp_raw;

    reg div_start; wire div_ready;
    reg signed [31:0] div_num, div_den;
    wire signed [31:0] div_res;

    div32_int u_div_rep (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .num_in(div_num), .den_in(div_den), .ready(div_ready), .result(div_res)
    );

    wire signed [31:0] dx32 = dx;
    wire signed [31:0] dy32 = dy;
    wire signed [31:0] r32  = obs_r;
    
    wire signed [31:0] term_Q18 = (dx32 * dx32) + (dy32 * dy32) - (r32 * r32);
    wire signed [15:0] term_raw = term_Q18 >>> 9;

    wire signed [31:0] aux_shifted = aux_Wp_raw >>> 4;
    wire signed [31:0] dx_scaled   = dx <<< 1;
    wire signed [31:0] dy_scaled   = dy <<< 1;
    // -------------------------------------------------------------

    localparam S_IDLE=0, S_REQ_X=1, S_WAIT_X=2, S_WAIT_Y=3, S_WAIT_R=4, S_WAIT_K=5;
    localparam S_CALC_DIST=6, S_CALC_TERM=7, S_DIV1=8, S_DIV1_WAIT=9;
    localparam S_DIV2=10, S_DIV2_WAIT=11, S_ACCUM=12, S_DONE=13;
    
    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0; div_start <= 0; ram_read_addr <= 0;
            W_acc <= 0; W_px_acc <= 0; W_py_acc <= 0; k_count <= 0;
            term_reg <= 0; term2_reg <= 0; aux_Wp_raw <= 0;
            obs_x <= 0; obs_y <= 0; obs_r <= 0; obs_k <= 0;
            dx <= 0; dy <= 0;
            W_acc_out <= 0; W_px_out <= 0; W_py_out <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start && sys_nc > 0) begin
                        k_count <= 0; W_acc <= 0; W_px_acc <= 0; W_py_acc <= 0;
                        state <= S_REQ_X;
                    end else if (start && sys_nc == 0) begin
                        done <= 1;
                    end
                end

                S_REQ_X: begin ram_read_addr <= (k_count << 2); state <= S_WAIT_X; end
                S_WAIT_X: begin ram_read_addr <= ram_read_addr + 1'b1; state <= S_WAIT_Y; end
                S_WAIT_Y: begin obs_x <= ram_read_data; ram_read_addr <= ram_read_addr + 1'b1; state <= S_WAIT_R; end
                S_WAIT_R: begin obs_y <= ram_read_data; ram_read_addr <= ram_read_addr + 1'b1; state <= S_WAIT_K; end
                S_WAIT_K: begin obs_r <= ram_read_data; state <= S_CALC_DIST; end

                S_CALC_DIST: begin
                    obs_k <= ram_read_data;
                    dx <= robot_x - obs_x;
                    dy <= robot_y - obs_y;
                    state <= S_CALC_TERM;
                end

                S_CALC_TERM: begin
                    if (term_raw < 16'sd2) begin
                        term_reg <= 16'sd2;
                        term2_reg <= 32'sd4;
                    end else begin
                        term_reg <= term_raw;
                        term2_reg <= term_raw * term_raw;
                    end
                    state <= S_DIV1;
                end

                S_DIV1: begin
                    div_num <= { {16{obs_k[15]}}, obs_k } <<< 12;
                    div_den <= { {16{term_reg[15]}}, term_reg };
                    div_start <= 1; state <= S_DIV1_WAIT;
                end
                S_DIV1_WAIT: begin
                    div_start <= 0;
                    if (div_ready) begin
                        W_acc <= W_acc + div_res;
                        state <= S_DIV2;
                    end
                end

                S_DIV2: begin
                    div_num <= -({ {16{obs_k[15]}}, obs_k } <<< 12);
                    div_den <= term2_reg;
                    div_start <= 1; state <= S_DIV2_WAIT;
                end
                S_DIV2_WAIT: begin
                    div_start <= 0;
                    if (div_ready) begin
                        aux_Wp_raw <= div_res;
                        state <= S_ACCUM;
                    end
                end

                S_ACCUM: begin
                    W_px_acc <= W_px_acc + ((aux_shifted * dx_scaled) >>> 5);
                    W_py_acc <= W_py_acc + ((aux_shifted * dy_scaled) >>> 5);
                    
                    if (k_count + 1 == sys_nc) begin
                        state <= S_DONE;
                    end else begin
                        k_count <= k_count + 1'b1;
                        state <= S_REQ_X; 
                    end
                end

                S_DONE: begin
                    if (W_px_acc > `GRAD_CLAMP) W_px_out <= `GRAD_CLAMP;
                    else if (W_px_acc < -`GRAD_CLAMP) W_px_out <= -`GRAD_CLAMP;
                    else W_px_out <= W_px_acc[15:0];

                    if (W_py_acc > `GRAD_CLAMP) W_py_out <= `GRAD_CLAMP;
                    else if (W_py_acc < -`GRAD_CLAMP) W_py_out <= -`GRAD_CLAMP;
                    else W_py_out <= W_py_acc[15:0];

                    W_acc_out <= W_acc;
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule 