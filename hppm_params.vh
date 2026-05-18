// hppm_params.vh
`ifndef HPPM_PARAMS_VH
`define HPPM_PARAMS_VH

// Punto fijo Q7.9 
`define FX_ONE            16'sd512     // 1.0 en Q7.9
`define FX_HALF           16'sd256     // 0.5 en Q7.9
`define FX_TINY           16'sd1       // epsilon

// UART 
`define CLKS_PER_BIT      5208         // 50 MHz / 9600 baud

// HPPM
`define NEWTON_MAX_ITERS  40           // Límite iteraciones Newton
`define NEWTON_PARO       16'sd2       // Umbral convergencia (~0.004 en Q7.9)
`define ARRIVAL_TOL       16'sd5       // Tolerancia llegada a meta (~0.01)
`define GRAD_CLAMP        32'sd4096    // Límite del gradiente repulsivo 

// Hardware
`define DIV_ITERS         7'd41        // Iteraciones para divisor fx_div32 

// Macros
`define FX_MUL_ROUND(a, b) ( ( ($signed(a) * $signed(b)) + `FX_HALF ) >>> 9 )

`endif // HPPM_PARAMS_VH

