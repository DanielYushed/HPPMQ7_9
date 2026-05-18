module hppm_stuck_recovery (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  wire [1:0]  irot_in,
    input  signed [31:0] last_dir_in,
    output reg  signed [31:0] rot_dir_out,

    input  signed [15:0] tray_prev_x, tray_prev_y, tray_prev_L,
    input  signed [15:0] trayC_curr_x, trayC_curr_y, trayC_curr_z,

    input  signed [15:0] xa, ya, La,
    input  signed [15:0] C1, C2, C3,
    input  signed [15:0] Dd0, Dd1, Dd2,

    input  signed [15:0] rad, signo, norxa,

    output reg         done,
    output reg         rever_flag,
    output reg  signed [15:0] new_x,
    output reg  signed [15:0] new_y
);

    localparam signed [15:0] TINY_FIX = 16'sd1;

    reg  norm_start; wire norm_done;
    reg  signed [15:0] nx_in, ny_in, nz_in;
    wire signed [15:0] norm_res;
    fx_norm3d u_norm (.clk(clk), .rst_n(rst_n), .start(norm_start), .x(nx_in), .y(ny_in), .z(nz_in), .done(norm_done), .norm_out(norm_res));

    reg  div_start; reg signed [31:0] div_num, div_den;
    wire div_ready; wire signed [15:0] div_res;
    fx_div32 u_div (.clk(clk), .rst_n(rst_n), .start(div_start), .num_in(div_num), .den_in(div_den), .ready(div_ready), .result(div_res));

    reg  signed [15:0] acos_in; wire signed [15:0] acos_out;
    fx_acos u_acos (.x(acos_in), .y(acos_out));

    reg  signed [31:0] rot_dir; wire signed [15:0] rx, ry;
    fx_rotacion u_rot (.x(Dd0 - C1), .y(Dd1 - C2), .direction(rot_dir), .xr(rx), .yr(ry));

    reg signed [15:0] t1_x, t1_y, t2_x, t2_y;
    reg signed [15:0] fi1;
    
    function [15:0] fx_abs;
        input [15:0] val;
        begin fx_abs = (val[15]) ? -val : val; end
    endfunction

    wire signed [31:0] mul_rad_norxa = rad * norxa;
    wire signed [15:0] term_rad_norxa = (mul_rad_norxa + 256) >>> 9;
    wire signed [31:0] mul_signo_term = signo * term_rad_norxa;
    wire signed [15:0] term_aux = (mul_signo_term + 256) >>> 9;

    wire signed [31:0] div_num_fi2 = {{16{term_aux[15]}}, term_aux};

    localparam S_IDLE     = 0, S_G1_NORM = 1,  S_G1_WAIT = 2,  S_G1_DIVX = 3,  S_G1_WX = 4,
               S_G1_DIVY  = 5, S_G1_WY   = 6,  S_G2_NORM = 7,  S_G2_WAIT = 8,  S_G2_DIVX = 9,
               S_G2_WX    = 10, S_G2_DIVY = 11, S_G2_WY = 12, S_CHECK_REV = 13,
               S_FI1_NORM = 14, S_FI1_W   = 15, S_FI1_DIV = 16, S_FI1_WDIV = 17,
               S_FI2_NORM = 18, S_FI2_W   = 19, S_FI2_DIV = 20, S_FI2_WDIV = 21, S_FINISH = 22;

    reg [4:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0; rever_flag <= 0;
            norm_start <= 0; div_start <= 0; rot_dir_out <= 0; rot_dir <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; rever_flag <= 0;
                    if (start) begin
                        nx_in <= (tray_prev_x - trayC_curr_x) <<< 1;
                        ny_in <= (tray_prev_y - trayC_curr_y) <<< 1;
                        nz_in <= (tray_prev_L - trayC_curr_z) <<< 1;
                        norm_start <= 1; state <= S_G1_WAIT;
                    end
                end
                
                S_G1_WAIT: begin
                    norm_start <= 0;
                    if (norm_done) begin
                        div_num <= {{16{nx_in[15]}}, nx_in};
                        div_den <= {{16{norm_res[15]}}, (norm_res < TINY_FIX) ? TINY_FIX : norm_res};
                        div_start <= 1; state <= S_G1_WX;
                    end
                end
                S_G1_WX: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; t1_x <= acos_out;
                        div_num <= {{16{ny_in[15]}}, ny_in}; 
                        div_start <= 1; state <= S_G1_WY;
                    end
                end
                S_G1_WY: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; t1_y <= acos_out;
                        nx_in <= (xa - C1) <<< 1; ny_in <= (ya - C2) <<< 1; nz_in <= (La - C3) <<< 1;
                        norm_start <= 1; state <= S_G2_WAIT;
                    end
                end

                S_G2_WAIT: begin
                    norm_start <= 0;
                    if (norm_done) begin
                        div_num <= {{16{nx_in[15]}}, nx_in};
                        div_den <= {{16{norm_res[15]}}, (norm_res < TINY_FIX) ? TINY_FIX : norm_res};
                        div_start <= 1; state <= S_G2_WX;
                    end
                end
                S_G2_WX: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; t2_x <= acos_out;
                        div_num <= {{16{ny_in[15]}}, ny_in}; 
                        div_start <= 1; state <= S_G2_WY;
                    end
                end
                S_G2_WY: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; t2_y <= acos_out;
                        state <= S_CHECK_REV;
                    end
                end

                S_CHECK_REV: begin
                    if (fx_abs(t1_x - t2_x) < 5 && fx_abs(t1_y - t2_y) < 5) begin
                        rever_flag <= 1;
                        nx_in <= trayC_curr_x - tray_prev_x;
                        ny_in <= trayC_curr_y - tray_prev_y;
                        nz_in <= trayC_curr_z - tray_prev_L;
                        norm_start <= 1; state <= S_FI1_W;
                    end else begin
                        rever_flag <= 0; done <= 1; state <= S_IDLE; 
                    end
                end

                S_FI1_W: begin
                    norm_start <= 0;
                    if (norm_done) begin
                        div_num <= {{16{nx_in[15]}}, nx_in};
                        div_den <= {{16{norm_res[15]}}, (norm_res < TINY_FIX) ? TINY_FIX : norm_res};
                        div_start <= 1; state <= S_FI1_WDIV;
                    end
                end
                
                S_FI1_WDIV: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; fi1 <= acos_out;
                        
                        if (irot_in == 2'd1) begin
                            nx_in <= Dd0 - trayC_curr_x; ny_in <= Dd1 - trayC_curr_y; nz_in <= Dd2 - trayC_curr_z;
                            norm_start <= 1; state <= S_FI2_W;
                        end else begin
                            rot_dir <= -last_dir_in;       
                            rot_dir_out <= -last_dir_in;   
                            state <= S_FINISH;             
                        end
                    end
                end

                S_FI2_W: begin
                    norm_start <= 0;
                    if (norm_done) begin
                        div_num <= div_num_fi2;
                        div_den <= {{16{norm_res[15]}}, (norm_res < TINY_FIX) ? TINY_FIX : norm_res};
                        div_start <= 1; state <= S_FI2_WDIV;
                    end
                end
                
                S_FI2_WDIV: begin
                    div_start <= 0;
                    if (div_ready) begin
                        acos_in <= div_res; 
                        rot_dir <= (acos_out > fi1) ? 32'd1 : -32'sd1;
                        rot_dir_out <= (acos_out > fi1) ? 32'd1 : -32'sd1;
                        state <= S_FINISH;
                    end
                end

                S_FINISH: begin
                    new_x <= rx + C1; 
                    new_y <= ry + C2;
                    done <= 1;
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule