module history_buffer (
    input clk, rst_n, shift_en,
    input signed [15:0] xa_in, ya_in, La_in,
    input signed [15:0] C1_in, C2_in, C3_in,

    output reg signed [15:0] tray_old_x, tray_old_y, tray_old_L,

    output wire signed [15:0] tray_curr_x, tray_curr_y, tray_curr_L,

    output reg signed [15:0] trayC_curr_x, trayC_curr_y, trayC_curr_z
);

    reg signed [15:0] t_x_1, t_y_1, t_L_1;
    reg signed [15:0] t_x_2, t_y_2, t_L_2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            t_x_1 <= 0; t_y_1 <= 0; t_L_1 <= 0;
            t_x_2 <= 0; t_y_2 <= 0; t_L_2 <= 0;
            tray_old_x <= 0; tray_old_y <= 0; tray_old_L <= 0;
            trayC_curr_x <= 0; trayC_curr_y <= 0; trayC_curr_z <= 0;
        end else if (shift_en) begin
            t_x_2 <= t_x_1; t_y_2 <= t_y_1; t_L_2 <= t_L_1;
            t_x_1 <= xa_in; t_y_1 <= ya_in; t_L_1 <= La_in;
            
            tray_old_x <= t_x_2; 
            tray_old_y <= t_y_2; 
            tray_old_L <= t_L_2;

            trayC_curr_x <= C1_in; 
            trayC_curr_y <= C2_in; 
            trayC_curr_z <= C3_in;
        end
    end

    assign tray_curr_x = t_x_1;
    assign tray_curr_y = t_y_1;
    assign tray_curr_L = t_L_1;

endmodule 