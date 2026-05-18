module newton_step (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    input  signed [15:0] J00, J01, J02,
    input  signed [15:0] J10, J11, J12,
    input  signed [15:0] J20, J21, J22,
    input  signed [15:0] f0, f1, f2,

    output reg         done,
    output reg  signed [15:0] delta_x,
    output reg  signed [15:0] delta_y,
    output reg  signed [15:0] delta_L
);

    function [15:0] fx_abs;
        input signed [15:0] val;
        begin fx_abs = (val[15]) ? -val : val; end
    endfunction

    function [15:0] fx_max3;
        input signed [15:0] a, b, c;
        reg [15:0] m1;
        begin
            m1 = (a > b) ? a : b;
            fx_max3 = (m1 > c) ? m1 : c;
        end
    endfunction

    wire [15:0] max_r0 = fx_max3(fx_abs(J00), fx_abs(J01), fx_abs(J02));
    wire [15:0] max_r1 = fx_max3(fx_abs(J10), fx_abs(J11), fx_abs(J12));

    wire [2:0] sh0 = (max_r0[14]) ? 3'd4 : (max_r0[13]) ? 3'd3 : 
                     (max_r0[12]) ? 3'd2 : (max_r0[11] && max_r0 > 16'd2048) ? 3'd1 : 3'd0;
                     
    wire [2:0] sh1 = (max_r1[14]) ? 3'd4 : (max_r1[13]) ? 3'd3 : 
                     (max_r1[12]) ? 3'd2 : (max_r1[11] && max_r1 > 16'd2048) ? 3'd1 : 3'd0;

    wire signed [15:0] sJ00 = J00 >>> sh0; wire signed [15:0] sJ01 = J01 >>> sh0; 
    wire signed [15:0] sJ02 = J02 >>> sh0; wire signed [15:0] sf0  = f0  >>> sh0;

    wire signed [15:0] sJ10 = J10 >>> sh1; wire signed [15:0] sJ11 = J11 >>> sh1; 
    wire signed [15:0] sJ12 = J12 >>> sh1; wire signed [15:0] sf1  = f1  >>> sh1;

    reg signed [15:0] sJ00_r, sJ01_r, sJ02_r, sf0_r;
    reg signed [15:0] sJ10_r, sJ11_r, sJ12_r, sf1_r;
    reg signed [15:0] sJ20_r, sJ21_r, sJ22_r, sf2_r;

    reg signed [15:0] t0_r, t1_r, t2_r;
    reg signed [15:0] c01_r, c02_r, c11_r, c12_r, c21_r, c22_r;

    reg signed [31:0] det_r;

    wire signed [31:0] det_long = ($signed(sJ00_r) * $signed(t0_r)) - ($signed(sJ01_r) * $signed(t1_r)) + ($signed(sJ02_r) * $signed(t2_r));
    wire signed [31:0] det_raw  = det_long >>> 9;
    wire signed [31:0] det_abs  = (det_raw[31]) ? -det_raw : det_raw;
    wire signed [31:0] det_safe = (det_abs < 1) ? ((det_raw[31]) ? -32'sd1 : 32'sd1) : det_raw;

    wire signed [15:0] adj [0:8];
    assign adj[0] = t0_r;  assign adj[1] = c01_r; assign adj[2] = c02_r;
    assign adj[3] = -t1_r; assign adj[4] = c11_r; assign adj[5] = c12_r;
    assign adj[6] = t2_r;  assign adj[7] = c21_r; assign adj[8] = c22_r;

    reg  div_start; wire div_ready;
    reg  signed [31:0] div_num, div_den;
    wire signed [31:0] div_res;
    
    div32_int u_div_step (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .num_in(div_num), .den_in(div_den), .ready(div_ready), .result(div_res)
    );

    reg [3:0] div_cnt;
    reg signed [31:0] Jinv [0:8];
    reg signed [31:0] dx_tmp, dy_tmp, dL_tmp;

    localparam S_IDLE       = 3'd0;
    localparam S_CALC_ADJ   = 3'd1;
    localparam S_CALC_DET   = 3'd2;
    localparam S_DIVIDE     = 3'd3;
    localparam S_WAIT       = 3'd4;
    localparam S_MULT_F     = 3'd5;
    
    reg [2:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 0; div_start <= 0; div_cnt <= 0;
            delta_x <= 0; delta_y <= 0; delta_L <= 0;
            dx_tmp <= 0; dy_tmp <= 0; dL_tmp <= 0;
            sJ00_r <= 0; sJ01_r <= 0; sJ02_r <= 0; sf0_r <= 0;
            sJ10_r <= 0; sJ11_r <= 0; sJ12_r <= 0; sf1_r <= 0;
            sJ20_r <= 0; sJ21_r <= 0; sJ22_r <= 0; sf2_r <= 0;
            t0_r <= 0; t1_r <= 0; t2_r <= 0; det_r <= 0;
            c01_r <= 0; c02_r <= 0; c11_r <= 0; c12_r <= 0; c21_r <= 0; c22_r <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        sJ00_r <= sJ00; sJ01_r <= sJ01; sJ02_r <= sJ02; sf0_r <= sf0;
                        sJ10_r <= sJ10; sJ11_r <= sJ11; sJ12_r <= sJ12; sf1_r <= sf1;
                        sJ20_r <= J20;  sJ21_r <= J21;  sJ22_r <= J22;  sf2_r <= f2;
                        state <= S_CALC_ADJ;
                    end
                end

                S_CALC_ADJ: begin
                    t0_r  <= (($signed(sJ11_r) * $signed(sJ22_r)) - ($signed(sJ12_r) * $signed(sJ21_r))) >>> 9;
                    t1_r  <= (($signed(sJ10_r) * $signed(sJ22_r)) - ($signed(sJ12_r) * $signed(sJ20_r))) >>> 9;
                    t2_r  <= (($signed(sJ10_r) * $signed(sJ21_r)) - ($signed(sJ11_r) * $signed(sJ20_r))) >>> 9;

                    c01_r <= -(($signed(sJ01_r) * $signed(sJ22_r)) - ($signed(sJ02_r) * $signed(sJ21_r))) >>> 9;
                    c02_r <=  (($signed(sJ01_r) * $signed(sJ12_r)) - ($signed(sJ02_r) * $signed(sJ11_r))) >>> 9;
                    
                    c11_r <=  (($signed(sJ00_r) * $signed(sJ22_r)) - ($signed(sJ02_r) * $signed(sJ20_r))) >>> 9;
                    c12_r <= -(($signed(sJ00_r) * $signed(sJ12_r)) - ($signed(sJ02_r) * $signed(sJ10_r))) >>> 9;

                    c21_r <= -(($signed(sJ00_r) * $signed(sJ21_r)) - ($signed(sJ01_r) * $signed(sJ20_r))) >>> 9;
                    c22_r <=  (($signed(sJ00_r) * $signed(sJ11_r)) - ($signed(sJ01_r) * $signed(sJ10_r))) >>> 9;
                    
                    state <= S_CALC_DET;
                end

                S_CALC_DET: begin
                    det_r <= det_safe;
                    div_cnt <= 0;
                    state <= S_DIVIDE;
                end

                S_DIVIDE: begin
                    div_num <= {{16{adj[div_cnt][15]}}, adj[div_cnt]}; 
                    div_den <= det_r;
                    div_start <= 1; state <= S_WAIT;
                end

                S_WAIT: begin
                    div_start <= 0;
                    if (div_ready) begin
                        Jinv[div_cnt] <= div_res;
                        if (div_cnt == 8) begin
                            state <= S_MULT_F;
                        end else begin
                            div_cnt <= div_cnt + 1'b1;
                            state <= S_DIVIDE;
                        end
                    end
                end

                S_MULT_F: begin
                    dx_tmp = (($signed(Jinv[0])*$signed(sf0_r)) + ($signed(Jinv[1])*$signed(sf1_r)) + ($signed(Jinv[2])*$signed(sf2_r))) >>> 9;
                    dy_tmp = (($signed(Jinv[3])*$signed(sf0_r)) + ($signed(Jinv[4])*$signed(sf1_r)) + ($signed(Jinv[5])*$signed(sf2_r))) >>> 9;
                    dL_tmp = (($signed(Jinv[6])*$signed(sf0_r)) + ($signed(Jinv[7])*$signed(sf1_r)) + ($signed(Jinv[8])*$signed(sf2_r))) >>> 9;

                    delta_x <= (dx_tmp > 32'sd12800) ? 16'sd12800 : (dx_tmp < -32'sd12800) ? -16'sd12800 : dx_tmp[15:0];
                    delta_y <= (dy_tmp > 32'sd12800) ? 16'sd12800 : (dy_tmp < -32'sd12800) ? -16'sd12800 : dy_tmp[15:0];
                    delta_L <= (dL_tmp > 32'sd12800) ? 16'sd12800 : (dL_tmp < -32'sd12800) ? -16'sd12800 : dL_tmp[15:0];
                    
                    done <= 1; state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule 