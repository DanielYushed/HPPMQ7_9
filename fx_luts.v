module fx_exp_neg (
    input  signed [15:0] x,
    output signed [15:0] y
);
    wire signed [15:0] x_pos = (x < 0) ? 16'sd0 : x;

    wire [15:0] idx = x_pos >>> 7;
    wire signed [15:0] resto = {9'd0, x_pos[6:0]};

    reg signed [15:0] y0;
    reg signed [15:0] y1;

    always @(*) begin
        case(idx[4:0]) 
            5'd0:  begin y0 = 512; y1 = 419; end
            5'd1:  begin y0 = 419; y1 = 343; end
            5'd2:  begin y0 = 343; y1 = 281; end
            5'd3:  begin y0 = 281; y1 = 230; end
            5'd4:  begin y0 = 230; y1 = 188; end
            5'd5:  begin y0 = 188; y1 = 154; end
            5'd6:  begin y0 = 154; y1 = 126; end
            5'd7:  begin y0 = 126; y1 = 103; end
            5'd8:  begin y0 = 103; y1 = 84;  end
            5'd9:  begin y0 = 84;  y1 = 69;  end
            5'd10: begin y0 = 69;  y1 = 56;  end
            5'd11: begin y0 = 56;  y1 = 46;  end
            5'd12: begin y0 = 46;  y1 = 38;  end
            5'd13: begin y0 = 38;  y1 = 31;  end
            5'd14: begin y0 = 31;  y1 = 25;  end
            5'd15: begin y0 = 25;  y1 = 21;  end
            default: begin y0 = 0; y1 = 0;   end 
        endcase
    end

    wire signed [15:0] safe_y0 = (idx >= 16) ? 16'sd0 : y0;
    wire signed [15:0] safe_y1 = (idx >= 16) ? 16'sd0 : y1;

    wire signed [31:0] diff = (safe_y1 - safe_y0) * resto;

    assign y = (idx >= 16) ? 16'sd0 : (safe_y0 + (diff >>> 7));

endmodule

module fx_acos (
    input  wire signed [15:0] x,
    output wire signed [15:0] y
);
    wire sign = x[15];
    wire [15:0] abs_x_raw = sign ? -x : x;
    
    wire [15:0] abs_x = (abs_x_raw > 16'd512) ? 16'd512 : abs_x_raw;

    wire [31:0] div_51 = abs_x * 32'd1286;
    wire [3:0] idx = div_51[19:16]; 

    reg [15:0] val;
    always @(*) begin
        case(idx)
            4'd0:  val = 804;
            4'd1:  val = 756;
            4'd2:  val = 707;
            4'd3:  val = 659;
            4'd4:  val = 609;
            4'd5:  val = 558;
            4'd6:  val = 505;
            4'd7:  val = 449;
            4'd8:  val = 389;
            4'd9:  val = 321;
            4'd10: val = 0;
            default: val = 0;
        endcase
    end

    localparam signed [15:0] PI_Q7_9 = 16'sd1608;
    assign y = sign ? (PI_Q7_9 - val) : val;

endmodule 