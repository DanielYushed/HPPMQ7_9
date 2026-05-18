`include "hppm_params.vh"

module fx_rotacion (
    input  signed [15:0] x,
    input  signed [15:0] y,
    input  signed [31:0] direction,
    output signed [15:0] xr,
    output signed [15:0] yr
);

    wire signed [15:0] c = 16'sd362;
    wire signed [15:0] s = (direction > 0) ? 16'sd362 : -16'sd362;

    assign xr = `FX_MUL_ROUND(x, c) - `FX_MUL_ROUND(y, s);
    assign yr = `FX_MUL_ROUND(x, s) + `FX_MUL_ROUND(y, c);

endmodule 