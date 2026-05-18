module fx_mul (
    input  signed [15:0] a,
    input  signed [15:0] b,
    output signed [15:0] result
);

    wire signed [31:0] product;

    assign product = a * b;

    assign result = (product + 32'sd256) >>> 9;

endmodule