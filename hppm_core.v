`include "hppm_params.vh"

module hppm_core (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        config_done,
    
    input  signed [15:0] sys_nc, sys_rr, sys_m1, sys_m2, sys_maxsteps,
    input  signed [15:0] sys_a0, sys_b0, sys_a1, sys_b1,
    
    output wire [9:0]  ram_read_addr,
    input  wire [15:0] ram_read_data,
    
    output wire        tx_pin,
    output reg         hppm_finished,
    output wire [4:0]  current_state 
);

    localparam S_HALT = 5'd18;

    reg signed [15:0] x_reg, y_reg, L_reg;
    reg signed [15:0] xa, ya, La;
    reg signed [15:0] C1, C2, C3;
    reg signed [15:0] Dd0, Dd1, Dd2;
    reg signed [15:0] rad, rad_prev, m_L1, signo; 

    reg [15:0] ii;
    reg cond1;

    wire signed [15:0] sx = (sys_a1 > sys_a0) ? `FX_ONE : -`FX_ONE;
    wire signed [15:0] sy = (sys_b1 > sys_b0) ? `FX_ONE : -`FX_ONE;
    
    wire lx = (sx == `FX_ONE) ? (xa >= (sys_a1 - `ARRIVAL_TOL)) : (xa <= (sys_a1 + `ARRIVAL_TOL));
    wire ly = (sy == `FX_ONE) ? (ya >= (sys_b1 - `ARRIVAL_TOL)) : (ya <= (sys_b1 + `ARRIVAL_TOL));

    reg signed [15:0] norxa_reg = `FX_ONE;
    reg signed [15:0] norya_reg = 16'sd0;

    reg  div_start;
    reg  signed [31:0] div_num;
    reg  signed [31:0] div_den;
    wire div_ready;
    wire signed [15:0] div_res;

    fx_div32 u_div_core (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .num_in(div_num), .den_in(div_den), .ready(div_ready), .result(div_res)
    );

    reg hist_shift;
    wire signed [15:0] t_old_x, t_old_y, t_old_L;
    wire signed [15:0] tC_curr_x, tC_curr_y, tC_curr_z;
    wire signed [15:0] t_curr_x, t_curr_y, t_curr_L; 

    history_buffer u_hist (
        .clk(clk), .rst_n(rst_n), .shift_en(hist_shift),
        .xa_in(xa), .ya_in(ya), .La_in(La), .C1_in(C1), .C2_in(C2), .C3_in(C3),
        .tray_old_x(t_old_x), .tray_old_y(t_old_y), .tray_old_L(t_old_L),
        .tray_curr_x(t_curr_x), .tray_curr_y(t_curr_y), .tray_curr_L(t_curr_L), 
        .trayC_curr_x(tC_curr_x), .trayC_curr_y(tC_curr_y), .trayC_curr_z(tC_curr_z)
    );

    reg signed [15:0] W_0_reg;
    reg signed [15:0] Q_reg;

    wire [4:0] state_wire; 
    
    wire signed [15:0] init_x = (state_wire == 5'd19 || state_wire == 5'd20) ? sys_a0 : sys_a1;
    wire signed [15:0] init_y = (state_wire == 5'd19 || state_wire == 5'd20) ? sys_b0 : sys_b1;

    reg init_rep_start;
    wire init_rep_done;
    wire signed [31:0] init_W_acc;
    
    wire newt_rep_start;
    wire newt_rep_done;
    wire signed [15:0] newt_rep_x, newt_rep_y;
    wire signed [31:0] shared_W_acc;
    wire signed [15:0] shared_W_px, shared_W_py;

    wire is_init_phase = (state_wire >= 5'd19 && state_wire <= 5'd22);
    
    wire shared_rep_start = is_init_phase ? init_rep_start : newt_rep_start;
    wire signed [15:0] shared_robot_x = is_init_phase ? init_x : newt_rep_x;
    wire signed [15:0] shared_robot_y = is_init_phase ? init_y : newt_rep_y;
    
    assign init_rep_done = is_init_phase ? newt_rep_done : 1'b0; 

    wire [9:0] shared_ram_addr;
    assign ram_read_addr = shared_ram_addr;

    repulsive_acc u_rep_shared (
        .clk(clk), .rst_n(rst_n), .start(shared_rep_start),
        .robot_x(shared_robot_x), .robot_y(shared_robot_y), .sys_nc(sys_nc),
        .ram_read_addr(shared_ram_addr), .ram_read_data(ram_read_data),
        .done(newt_rep_done), .W_acc_out(shared_W_acc),
        .W_px_out(shared_W_px), .W_py_out(shared_W_py) 
    );
    
    assign init_W_acc = shared_W_acc;

    wire newt_norm_start, up_norm_start;
    wire newt_norm_done, up_norm_done, core_norm_done;
    wire signed [15:0] newt_norm_x, newt_norm_y, newt_norm_z;
    wire signed [15:0] up_norm_x, up_norm_y, up_norm_z;
    wire signed [15:0] shared_norm_res;
    
    reg norm_start; 

    wire is_newt_running = (state_wire == 5'd2); // S_WAIT_NEWT
    wire is_up_running   = (state_wire == 5'd6); // S_WAIT_UPDT

    wire shared_norm_start = norm_start | newt_norm_start | up_norm_start;

    wire signed [15:0] shared_norm_x = is_newt_running ? newt_norm_x :
                                       is_up_running   ? up_norm_x :
                                       (t_old_x - tC_curr_x);

    wire signed [15:0] shared_norm_y = is_newt_running ? newt_norm_y :
                                       is_up_running   ? up_norm_y :
                                       (t_old_y - tC_curr_y);

    wire signed [15:0] shared_norm_z = is_newt_running ? newt_norm_z :
                                       is_up_running   ? up_norm_z :
                                       (t_old_L - tC_curr_z);

    fx_norm3d u_shared_norm (
        .clk(clk), .rst_n(rst_n), .start(shared_norm_start),
        .x(shared_norm_x), .y(shared_norm_y), .z(shared_norm_z),
        .done(core_norm_done), .norm_out(shared_norm_res)
    );

    assign newt_norm_done = is_newt_running ? core_norm_done : 1'b0;
    assign up_norm_done   = is_up_running   ? core_norm_done : 1'b0;

    reg recov_start; wire recov_done, recov_rever;
    wire signed [15:0] recov_x, recov_y;
    reg [1:0] irot_reg;
    reg signed [31:0] last_direction;
    wire signed [31:0] rot_dir_cable;

    hppm_stuck_recovery u_recov (
        .clk(clk), .rst_n(rst_n), .start(recov_start),
        .irot_in(irot_reg), .last_dir_in(last_direction), .rot_dir_out(rot_dir_cable),
        .tray_prev_x(t_old_x), .tray_prev_y(t_old_y), .tray_prev_L(t_old_L),
        .trayC_curr_x(tC_curr_x), .trayC_curr_y(tC_curr_y), .trayC_curr_z(tC_curr_z),
        .xa(xa), .ya(ya), .La(La), .C1(C1), .C2(C2), .C3(C3),
        .Dd0(Dd0), .Dd1(Dd1), .Dd2(Dd2),
        .rad(rad), .signo(signo), .norxa(norxa_reg),
        .done(recov_done), .rever_flag(recov_rever),
        .new_x(recov_x), .new_y(recov_y)
    );

    reg newt_start; wire newt_done;
    wire signed [15:0] newt_x_out, newt_y_out, newt_L_out;
    wire [15:0] newt_iters;
    wire signed [15:0] J00, J01, J02, J10, J11, J12; 
    
    newton_inner_loop u_newt_loop (
        .clk(clk), .rst_n(rst_n), .start(newt_start),
        .x_in(Dd0), .y_in(Dd1), .L_in(Dd2), 
        .C1_in(C1), .C2_in(C2), .C3_in(C3),
        .rad_in(rad), .W_0_in(W_0_reg), .Q_in(Q_reg),     
        .sys_nc(sys_nc), .sys_m1(sys_m1), .sys_m2(sys_m2), 
        .sys_a0(sys_a0), .sys_b0(sys_b0), .sys_a1(sys_a1), .sys_b1(sys_b1),
        .sys_paro(`NEWTON_PARO), 
        
        .rep_start_out(newt_rep_start), .rep_done_in(newt_rep_done),
        .rep_robot_x_out(newt_rep_x), .rep_robot_y_out(newt_rep_y),
        .rep_W_acc_in(shared_W_acc), .rep_W_px_in(shared_W_px), .rep_W_py_in(shared_W_py),

        .norm_start_out(newt_norm_start), .norm_done_in(newt_norm_done),
        .norm_x_out(newt_norm_x), .norm_y_out(newt_norm_y), .norm_z_out(newt_norm_z),
        .norm_res_in(shared_norm_res),
        
        .done(newt_done), .x_out(newt_x_out), .y_out(newt_y_out), .L_out(newt_L_out), .iters_out(newt_iters),
        .J00_out(J00), .J01_out(J01), .J02_out(J02), .J10_out(J10), .J11_out(J11), .J12_out(J12)
    );

    reg up_start; wire up_done;
    wire signed [15:0] up_C1, up_C2, up_C3, up_Dd0, up_Dd1, up_Dd2, up_rad, up_mL1, up_norxa, up_norya;
    
    hypersphere_updater u_updater (
        .clk(clk), .rst_n(rst_n), .start(up_start),
        .J00(J00), .J01(J01), .J02(J02), .J10(J10), .J11(J11), .J12(J12),
        .xa(xa), .ya(ya), .La(La), 
        .x_old(t_curr_x), .y_old(t_curr_y), .L_old(t_curr_L),
        .sys_rr(sys_rr), .r_prev(rad_prev), .m_L1_in(m_L1), .signo(signo),
        
        .norm_start_out(up_norm_start), .norm_done_in(up_norm_done),
        .norm_x_out(up_norm_x), .norm_y_out(up_norm_y), .norm_z_out(up_norm_z),
        .norm_res_in(shared_norm_res),
        
        .done(up_done), .C1_out(up_C1), .C2_out(up_C2), .C3_out(up_C3),
        .Dd0_out(up_Dd0), .Dd1_out(up_Dd1), .Dd2_out(up_Dd2), .rad_out(up_rad), .m_L1_out(up_mL1),
        .norxa_out(up_norxa), .norya_out(up_norya)
    );

    reg tx_start; wire tx_done; wire tx_uart_act, tx_uart_dn; wire [7:0] tx_uart_dt;
    tx_formatter u_txf (
        .clk(clk), .rst_n(rst_n), .start(tx_start), 
        .x_in(xa), .y_in(ya), .L_in(La), 
        .tx_done(tx_uart_dn), .tx_start(tx_uart_act), .tx_data(tx_uart_dt), 
        .done(tx_done) 
    );
    
    wire tx_dummy;
    uart_tx #(.CLKS_PER_BIT(`CLKS_PER_BIT)) u_uart (
        .clk(clk), .rst_n(rst_n), .tx_start(tx_uart_act), 
        .tx_data(tx_uart_dt), .tx_pin(tx_pin), .tx_active(tx_dummy), .tx_done(tx_uart_dn)
    ); 

    wire signed [31:0] mul_m1_a0 = sys_m1 * sys_a0;
    wire signed [31:0] mul_m1_a1 = sys_m1 * sys_a1;
    wire signed [15:0] fx_m1_a0 = (mul_m1_a0 + `FX_HALF) >>> 9;
    wire signed [15:0] fx_m1_a1 = (mul_m1_a1 + `FX_HALF) >>> 9;
    
    wire signed [15:0] term_L_raw = sys_b1 + fx_m1_a1 - fx_m1_a0 - sys_b0;

    wire signed [15:0] term_L_abs = (term_L_raw[15]) ? -term_L_raw : term_L_raw;
    wire signed [15:0] term_L = (term_L_abs < `FX_TINY) ? `FX_TINY : term_L_raw;

    wire signed [31:0] mul_m1_x_arranque = sys_m1 * Dd0; 
    wire signed [15:0] fx_m1_x_arranque = (mul_m1_x_arranque + `FX_HALF) >>> 9;
    wire signed [15:0] num_L = Dd1 + fx_m1_x_arranque - sys_b0 - fx_m1_a0;

    localparam S_IDLE            = 5'd0,  S_PRELOAD_HIST    = 5'd17,
               S_START_NEWT      = 5'd1,  S_WAIT_NEWT       = 5'd2,
               S_WAIT_ATAS       = 5'd4,  S_START_UPDT      = 5'd5,
               S_WAIT_UPDT       = 5'd6,  S_UPDATE_GLOB     = 5'd7,
               S_START_TX        = 5'd8,  S_WAIT_TX         = 5'd9,
               S_CHECK_LOOP      = 5'd10, S_CALC_ARRANQUE_L = 5'd14, 
               S_WAIT_ARRANQUE_L = 5'd15, S_WAIT_RECOV      = 5'd16,
               S_INIT_W0         = 5'd19, S_WAIT_W0         = 5'd20,
               S_INIT_Q          = 5'd21, S_WAIT_Q          = 5'd22,
               S_START_FINAL_TX  = 5'd23, S_WAIT_FINAL_TX   = 5'd24;

    reg [4:0] state; 
    assign state_wire = state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; hppm_finished <= 0;
            newt_start <= 0; up_start <= 0; tx_start <= 0; hist_shift <= 0; 
            norm_start <= 0; recov_start <= 0; init_rep_start <= 0;
            irot_reg <= 0; last_direction <= 0; cond1 <= 0;
            W_0_reg <= 0; Q_reg <= 0; div_start <= 0;
            
            norxa_reg <= `FX_ONE;
            norya_reg <= 16'sd0;
        end else begin
            case (state)
                S_IDLE: begin
                    hppm_finished <= 1'b0; 
                    if (config_done) begin
                        ii <= 1;
                        rad <= sys_rr;
                        rad_prev <= sys_rr;
                        signo <= (sys_m1 > sys_m2) ? -`FX_ONE : `FX_ONE;
                        Dd0 <= sys_a0 + sys_rr; 
                        Dd1 <= sys_b0;
                        cond1 <= 0;
                        
                        xa <= sys_a0; ya <= sys_b0; La <= 16'sd0; 
                        C1 <= sys_a0; C2 <= sys_b0; C3 <= 16'sd0;
                        
                        state <= S_INIT_W0; 
                    end
                end

                S_INIT_W0: begin init_rep_start <= 1; state <= S_WAIT_W0; end
                
                S_WAIT_W0: begin
                    init_rep_start <= 0;
                    if (init_rep_done) begin
                        W_0_reg <= (init_W_acc + {16'sd0, `FX_HALF}) >>> 9; 
                        state <= S_INIT_Q;
                    end
                end

                S_INIT_Q: begin init_rep_start <= 1; state <= S_WAIT_Q; end
                
                S_WAIT_Q: begin
                    init_rep_start <= 0;
                    if (init_rep_done) begin
                        Q_reg <= (init_W_acc + {16'sd0, `FX_HALF}) >>> 9; 
                        hist_shift <= 1; 
                        state <= S_PRELOAD_HIST; 
                    end
                end

                S_PRELOAD_HIST: begin hist_shift <= 0; state <= S_CALC_ARRANQUE_L; end
                
                S_CALC_ARRANQUE_L: begin
                    div_num <= {{16{num_L[15]}}, num_L};
                    div_den <= {{16{term_L[15]}}, term_L};
                    div_start <= 1;
                    state <= S_WAIT_ARRANQUE_L;
                end
                
                S_WAIT_ARRANQUE_L: begin
                    div_start <= 0;
                    if (div_ready) begin
                        Dd2 <= div_res[15:0];
                        if (ii == 1 && div_res[15:0] < La) begin
                            signo <= -signo;
                            Dd0 <= sys_a0 + `FX_MUL_ROUND(-signo, `FX_MUL_ROUND(rad, norxa_reg));
                            Dd1 <= sys_b0 + `FX_MUL_ROUND(-signo, `FX_MUL_ROUND(rad, norya_reg));
                            ii <= 2; 
                            state <= S_CALC_ARRANQUE_L; 
                        end else begin
                            state <= S_START_NEWT; 
                        end
                    end
                end

                S_START_NEWT: begin hist_shift <= 1; newt_start <= 1; state <= S_WAIT_NEWT; end
                
                S_WAIT_NEWT: begin
                    hist_shift <= 0; newt_start <= 0;
                    if (newt_done) begin
                        xa <= newt_x_out; ya <= newt_y_out; La <= newt_L_out;
                        if (ii > 3) begin
                            norm_start <= 1;
                            state <= S_WAIT_ATAS;
                        end else begin
                            state <= S_START_UPDT;
                        end
                    end
                end

                S_WAIT_ATAS: begin
                    norm_start <= 0;
                    if (core_norm_done) begin
                        if (shared_norm_res > `ARRIVAL_TOL) begin 
                            if (irot_reg == 2'd0) begin irot_reg <= 2'd1; recov_start <= 1; state <= S_WAIT_RECOV; end 
                            else if (irot_reg < 2'd3) begin recov_start <= 1; state <= S_WAIT_RECOV; end 
                            else begin irot_reg <= 2'd0; state <= S_START_UPDT; end
                        end else begin
                            irot_reg <= 2'd0; state <= S_START_UPDT;
                        end
                    end
                end

                S_WAIT_RECOV: begin
                    recov_start <= 0;
                    if (recov_done) begin
                        if (recov_rever == 1'b1 && irot_reg < 2'd3) begin
                            Dd0 <= recov_x; Dd1 <= recov_y;
                            last_direction <= rot_dir_cable;
                            irot_reg <= irot_reg + 1'b1; 
                            state <= S_CALC_ARRANQUE_L;
                        end else begin
                            irot_reg <= 2'd0; state <= S_START_UPDT;
                        end
                    end
                end

                S_START_UPDT: begin up_start <= 1; state <= S_WAIT_UPDT; end
                
                S_WAIT_UPDT: begin
                    up_start <= 0;
                    if (up_done) begin
                        C1 <= up_C1; C2 <= up_C2; C3 <= up_C3; Dd0 <= up_Dd0; Dd1 <= up_Dd1; Dd2 <= up_Dd2;
                        rad_prev <= rad; rad <= up_rad; m_L1 <= up_mL1; norxa_reg <= up_norxa; norya_reg <= up_norya;
                        state <= S_UPDATE_GLOB;
                    end
                end

                S_UPDATE_GLOB: begin
                    if (La >= `FX_ONE && lx && ly) begin
                        cond1 <= 1;
                        xa <= sys_a1;
                        ya <= sys_b1;
                        La <= `FX_ONE;
                    end
                    
                    ii <= ii + 1'b1;
                    state <= S_START_TX;
                end

                S_START_TX: begin tx_start <= 1; state <= S_WAIT_TX; end
                
                S_WAIT_TX: begin
                    tx_start <= 0;
                    if (tx_done == 1'b1) state <= S_CHECK_LOOP;
                end

                S_CHECK_LOOP: begin
                    if ((ii < sys_maxsteps) && (La > -`FX_HALF) && (cond1 == 0)) begin
                        state <= S_START_NEWT;
                    end else begin
                        state <= S_START_FINAL_TX; 
                    end
                end
                
                S_START_FINAL_TX: begin
                    tx_start <= 1;
                    state <= S_WAIT_FINAL_TX;
                end
                
                S_WAIT_FINAL_TX: begin
                    tx_start <= 0;
                    if (tx_done == 1'b1) begin
                        hppm_finished <= 1;
                        state <= S_HALT;
                    end
                end
                // ------------------------------------------------------
                
                S_HALT: begin 
                    hppm_finished <= 1'b1; 
                    if (config_done == 1'b0) begin
                        state <= S_IDLE;
                    end
                end
                    
                default: begin
                    state <= S_IDLE; hppm_finished <= 0; newt_start <= 0; up_start <= 0; tx_start <= 0; 
                    hist_shift <= 0; norm_start <= 0; recov_start <= 0; init_rep_start <= 0; div_start <= 0;
                end
            endcase
        end
    end
endmodule 