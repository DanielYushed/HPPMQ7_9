#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h> 

typedef int16_t fx16;
typedef int32_t fx32;
typedef int64_t fx64; 

#define Q_SHIFT 9
#define K_ONE   512     
#define MAX_FX  32767
#define MIN_FX  -32768
#define PI_FIX  1608    
#define TINY_FIX 1      

#define HW_OBS_CLAMP     2      // Evita colapso por división por cero
#define MAX_GRAD_WP_TOT  4096   // Fuerza máxima permitida 
#define ROW_TARGET_Q79   2048   // Target de escalamiento de matriz

fx16 float_to_fix(double f) {
    double s = f * K_ONE;
    if (s >= MAX_FX) return MAX_FX;
    if (s <= MIN_FX) return MIN_FX;
    return (fx16)(s + (s >= 0 ? 0.5 : -0.5));
}

double fix_to_float(fx16 q) { return (double)q / K_ONE; }

fx16 fx_mul(fx16 a, fx16 b) {
    fx32 res = (fx32)a * (fx32)b;
    return (fx16)((res + (1 << (Q_SHIFT - 1))) >> Q_SHIFT);
}

fx16 fx_div32(fx32 a, fx32 b) {
    if (b == 0) return (a >= 0) ? MAX_FX : MIN_FX;
    fx64 res = ((fx64)a << Q_SHIFT) / b;
    if (res > MAX_FX) return MAX_FX;
    if (res < MIN_FX) return MIN_FX;
    return (fx16)res;
}

fx16 fx_div(fx16 a, fx16 b) { return fx_div32((fx32)a, (fx32)b); }
fx16 fx_sqr(fx16 a) { return fx_mul(a, a); }
fx16 fx_abs(fx16 a) { return (a < 0) ? -a : a; }

fx16 fx_norm3d(fx16 x, fx16 y, fx16 z) {
    fx32 sum_Q18 = ((fx32)x*x) + ((fx32)y*y) + ((fx32)z*z);
    if (sum_Q18 <= 0) return 0;
    
    uint32_t val = sum_Q18; 
    uint32_t res = 0; 
    uint32_t bit = 1 << 30; 
    while (bit > val) { bit >>= 2; }
    while (bit != 0) {
        if (val >= res + bit) { val -= res + bit; res = (res >> 1) + bit; } 
        else { res >>= 1; }
        bit >>= 2;
    } return (fx16)res;
}

const fx16 LUT_EXP[] = { 512, 419, 343, 281, 230, 188, 154, 126, 103, 84, 69, 56, 46, 38, 31, 25, 21 };

fx16 fx_exp_neg(fx16 x) {
    if (x < 0) { x = 0; }
    int idx = x >> 7; 
    if (idx >= 16) return 0;
    fx16 y0 = LUT_EXP[idx]; fx16 y1 = LUT_EXP[idx+1];
    int resto = x & 0x7F; fx32 diff = (fx32)(y1 - y0) * resto;
    return y0 + (fx16)(diff >> 7);
}

const fx16 LUT_ACOS[] = { 804, 756, 707, 659, 609, 558, 505, 449, 389, 321, 0 }; 

fx16 fx_acos(fx16 x) {
    if (x > K_ONE) { x = K_ONE; }
    if (x < -K_ONE) { x = -K_ONE; }
    int neg = 0; if (x < 0) { x = -x; neg = 1; } 
    int idx = x / 51; if (idx > 10) { idx = 10; }
    fx16 val = LUT_ACOS[idx]; return neg ? PI_FIX - val : val;
}

void fx_rotacion(fx16 x, fx16 y, int direction, fx16 *xr, fx16 *yr) {
    fx16 c = 362; fx16 s = (direction > 0) ? 362 : -362;
    *xr = fx_mul(x, c) - fx_mul(y, s); *yr = fx_mul(x, s) + fx_mul(y, c);
}

typedef struct {
    int nc, maxsteps;
    fx16 rr, m1, m2, paro;
    fx16 a0, b0, a1, b1;
    fx16 mc[200][4]; 
} Sistema;

void equilibrar_filas_matriz(fx16 J[3][3], fx16 f[3]) {
    fx16 mx0 = fx_abs(J[0][0]);
    if (fx_abs(J[0][1]) > mx0) mx0 = fx_abs(J[0][1]);
    if (fx_abs(J[0][2]) > mx0) mx0 = fx_abs(J[0][2]);
    int k0 = 0; while (k0 < 8 && mx0 > ROW_TARGET_Q79) { mx0 >>= 1; k0++; }
    if (k0 > 0) { J[0][0] >>= k0; J[0][1] >>= k0; J[0][2] >>= k0; f[0] >>= k0; }

    fx16 mx1 = fx_abs(J[1][0]);
    if (fx_abs(J[1][1]) > mx1) mx1 = fx_abs(J[1][1]);
    if (fx_abs(J[1][2]) > mx1) mx1 = fx_abs(J[1][2]);
    int k1 = 0; while (k1 < 8 && mx1 > ROW_TARGET_Q79) { mx1 >>= 1; k1++; }
    if (k1 > 0) { J[1][0] >>= k1; J[1][1] >>= k1; J[1][2] >>= k1; f[1] >>= k1; }
}

void invertir_3x3_fix(fx16 J[3][3], fx16 Ji[3][3]) {
    fx16 t0 = (fx16)(((fx32)J[1][1]*J[2][2] - (fx32)J[1][2]*J[2][1]) >> Q_SHIFT);
    fx16 t1 = (fx16)(((fx32)J[1][0]*J[2][2] - (fx32)J[1][2]*J[2][0]) >> Q_SHIFT);
    fx16 t2 = (fx16)(((fx32)J[1][0]*J[2][1] - (fx32)J[1][1]*J[2][0]) >> Q_SHIFT);
    
    fx32 det = ((fx32)J[0][0]*t0 - (fx32)J[0][1]*t1 + (fx32)J[0][2]*t2) >> Q_SHIFT;
    
    if (fx_abs((fx16)det) < TINY_FIX) { det = (det < 0) ? -TINY_FIX : TINY_FIX; }
    
    Ji[0][0] = fx_div32(t0, det); Ji[1][0] = fx_div32(-t1, det); Ji[2][0] = fx_div32(t2, det); 
    
    fx16 c01 = (fx16)(-((fx32)J[0][1]*J[2][2] - (fx32)J[0][2]*J[2][1]) >> Q_SHIFT);
    fx16 c02 = (fx16)( ((fx32)J[0][1]*J[1][2] - (fx32)J[0][2]*J[1][1]) >> Q_SHIFT);
    Ji[0][1] = fx_div32(c01, det); Ji[0][2] = fx_div32(c02, det);
    
    fx16 c11 = (fx16)( ((fx32)J[0][0]*J[2][2] - (fx32)J[0][2]*J[2][0]) >> Q_SHIFT);
    fx16 c12 = (fx16)(-((fx32)J[0][0]*J[1][2] - (fx32)J[0][2]*J[1][0]) >> Q_SHIFT);
    Ji[1][1] = fx_div32(c11, det); Ji[1][2] = fx_div32(c12, det);
    
    fx16 c21 = (fx16)(-((fx32)J[0][0]*J[2][1] - (fx32)J[0][1]*J[2][0]) >> Q_SHIFT);
    fx16 c22 = (fx16)( ((fx32)J[0][0]*J[1][1] - (fx32)J[0][1]*J[1][0]) >> Q_SHIFT);
    Ji[2][1] = fx_div32(c21, det); Ji[2][2] = fx_div32(c22, det);
}

void calc_f_jacob_fix(Sistema *s, fx16 x, fx16 y, fx16 L, fx16 C1, fx16 C2, fx16 C3, 
                      fx16 rad, fx16 W_0, fx16 Q, fx16 f[3], fx16 jacob[3][3]) {
    fx32 W_acc = 0;
    fx32 W_px_acc = 0, W_py_acc = 0; 
    
    for (int k = 0; k < s->nc; k++) {
        fx16 dx = x - s->mc[k][0];
        fx16 dy = y - s->mc[k][1];
        
        fx32 dx32 = dx; fx32 dy32 = dy; fx32 rad32 = s->mc[k][2];
        fx32 term_Q18 = (dx32*dx32) + (dy32*dy32) - (rad32*rad32);
        fx16 term = (fx16)(term_Q18 >> 9);
        
        if (term < HW_OBS_CLAMP) { term = HW_OBS_CLAMP; }
        
        fx32 k_amp = (fx32)s->mc[k][3];
        W_acc += (k_amp << 12) / term; 

        fx32 term2_32 = (fx32)term * (fx32)term; 
        fx32 aux_Wp_raw = -((k_amp << 12) / term2_32);
        
        fx32 Wpx_part = ((aux_Wp_raw >> 4) * (2*dx)) >> (Q_SHIFT - 4);
        fx32 Wpy_part = ((aux_Wp_raw >> 4) * (2*dy)) >> (Q_SHIFT - 4);
        
        W_px_acc += Wpx_part; 
        W_py_acc += Wpy_part;
    }
    
    if (W_px_acc > MAX_GRAD_WP_TOT) { W_px_acc = MAX_GRAD_WP_TOT; } 
    if (W_px_acc < -MAX_GRAD_WP_TOT) { W_px_acc = -MAX_GRAD_WP_TOT; }
    if (W_py_acc > MAX_GRAD_WP_TOT) { W_py_acc = MAX_GRAD_WP_TOT; } 
    if (W_py_acc < -MAX_GRAD_WP_TOT) { W_py_acc = -MAX_GRAD_WP_TOT; }

    jacob[0][0] = -s->m1;
    jacob[0][1] = -K_ONE;
    jacob[0][2] = -s->b0 - fx_mul(s->m1, s->a0) + fx_mul(s->m1, s->a1) + s->b1;

    jacob[1][0] = -s->m2 + (fx16)W_px_acc;
    jacob[1][1] = -K_ONE + (fx16)W_py_acc;
    jacob[1][2] = -s->b0 - fx_mul(s->m2, s->a0) + s->b1 + fx_mul(s->m2, s->a1) + W_0 - Q;

    fx16 dx_c = x - C1; 
    fx16 dy_c = y - C2; 
    fx16 dL_c = L - C3;

    jacob[2][0] = dx_c << 1;
    jacob[2][1] = dy_c << 1;
    jacob[2][2] = dL_c << 1;
    
    fx16 W = (fx16)(W_acc >> 9);
    fx16 term_recta1 = -s->b0 - fx_mul(s->m1, s->a0) + fx_mul(s->m1, s->a1) + s->b1;
    f[0] = -y - fx_mul(s->m1, x) + fx_mul(s->m1, s->a1) + s->b1 - fx_mul(K_ONE - L, term_recta1);
    
    fx16 term_recta2 = -s->b0 - fx_mul(s->m2, s->a0) + (s->b1 + fx_mul(s->m2, s->a1)) + W_0 - Q;
    f[1] = (-y - fx_mul(s->m2, x) + (s->b1 + fx_mul(s->m2, s->a1)) + W - Q) - fx_mul(K_ONE - L, term_recta2);
    
    // f2 = dx^2 + dy^2 + dL^2 - rad^2
    fx32 dCx_32 = dx_c; fx32 dCy_32 = dy_c; fx32 dCL_32 = dL_c; fx32 rad_32 = rad;
    fx32 f2_Q18 = (dCx_32 * dCx_32) + (dCy_32 * dCy_32) + (dCL_32 * dCL_32) - (rad_32 * rad_32);
    f[2] = (fx16)(f2_Q18 >> Q_SHIFT);
}

int main() {
    Sistema s; 
    memset(&s, 0, sizeof(Sistema)); 
    s.paro = 2; 

    FILE *fp1 = fopen("pendientes.txt", "r"); 
    if (!fp1) { printf("Error: Archivo pendientes no encontrado.\n"); return 1; }
    char buf[100];
    
    fscanf(fp1, "%s", buf); s.rr = float_to_fix(atof(buf));
    if(s.rr < 5) s.rr = 5;
    fscanf(fp1, "%s", buf); s.m1 = float_to_fix(atof(buf));
    fscanf(fp1, "%s", buf); s.m2 = float_to_fix(atof(buf));
    fscanf(fp1, "%s", buf); s.maxsteps = atoi(buf);
    
    fscanf(fp1, "%s", buf); s.a0 = float_to_fix(atof(buf));
    fscanf(fp1, "%s", buf); s.b0 = float_to_fix(atof(buf));
    fscanf(fp1, "%s", buf); s.a1 = float_to_fix(atof(buf));
    fscanf(fp1, "%s", buf); s.b1 = float_to_fix(atof(buf));
    fclose(fp1);

    FILE *fp = fopen("obstaculos1.txt", "r");
    s.nc = 0; 
    
    if (fp) {
        double v0, v1, v2, v3;
        while ((s.nc < 200) && (fscanf(fp, "%lf %lf %lf %lf", &v0, &v1, &v2, &v3) == 4)) {
            s.mc[s.nc][0] = float_to_fix(v0); 
            s.mc[s.nc][1] = float_to_fix(v1);
            s.mc[s.nc][2] = float_to_fix(v2); 
            s.mc[s.nc][3] = float_to_fix(v3); 
            s.nc++; 
        } 
        fclose(fp);
    } else { 
        printf("Error: Archivo obstaculos no encontrado.\n"); 
        return 1; 
    }

    fx16 x = s.a0, y = s.b0, r = s.rr, rad = s.rr;
    fx16 term_L = (s.b1 + fx_mul(s.m1, s.a1 - s.a0) - s.b0);
    if (fx_abs(term_L) < TINY_FIX) { term_L = TINY_FIX; }

    fx16 L = fx_div(y + fx_mul(s.m1, x) - (s.b0 + fx_mul(s.m1, s.a0)), term_L);
    fx16 xa = x, ya = y, La = L;

    static fx16 tray[5010][3]; 
    fx16 trayC[3][3] = {0};
    int ii = 1, cond1 = 0, inr = 0; 
    fx32 Q_acc = 0, W0_acc = 0; 
    fx16 norxa = K_ONE, norya = 0, norLa = 0;

    for(int k=0; k < s.nc; k++) {
        fx16 dx = s.a1 - s.mc[k][0]; fx16 dy = s.b1 - s.mc[k][1]; fx16 mrad = s.mc[k][2];
        fx32 dQ_Q18 = (fx32)dx*dx + (fx32)dy*dy - (fx32)mrad*mrad;
        fx16 dQ = (fx16)(dQ_Q18 >> 9); if(dQ <= TINY_FIX) { dQ = TINY_FIX; }
        fx32 k_amp = (fx32)s.mc[k][3]; Q_acc += (k_amp << 12) / dQ; 
        
        fx16 dx0 = s.a0 - s.mc[k][0]; fx16 dy0 = s.b0 - s.mc[k][1];
        fx32 dW0_Q18 = (fx32)dx0*dx0 + (fx32)dy0*dy0 - (fx32)mrad*mrad;
        fx16 dW0 = (fx16)(dW0_Q18 >> 9); if(dW0 <= TINY_FIX) { dW0 = TINY_FIX; }
        W0_acc += (k_amp << 12) / dW0; 
    }
    fx16 Q = (fx16)(Q_acc >> 9); fx16 W_0 = (fx16)(W0_acc >> 9); fx16 m_L1 = K_ONE;

    fx16 C1 = xa, C2 = ya, C3 = La; 
    fx16 Dd[3];
    fx16 signo = (s.m1 > s.m2) ? -K_ONE : K_ONE;

    fx16 f_init[3], jacob_init[3][3];
    calc_f_jacob_fix(&s, xa, ya, La, C1, C2, C3, rad, W_0, Q, f_init, jacob_init);
    fx32 det32_init = -((fx32)jacob_init[0][0]*jacob_init[1][1] - (fx32)jacob_init[0][1]*jacob_init[1][0]) >> Q_SHIFT;
    fx32 ad1_32_init = ((fx32)jacob_init[0][2]*jacob_init[1][1] - (fx32)jacob_init[1][2]*jacob_init[0][1]) >> Q_SHIFT;
    fx32 ad2_32_init = ((fx32)jacob_init[1][2]*jacob_init[0][0] - (fx32)jacob_init[0][2]*jacob_init[1][0]) >> Q_SHIFT;
    fx16 nBA_init = fx_norm3d((fx16)ad1_32_init, (fx16)ad2_32_init, (fx16)det32_init);
    if (nBA_init < TINY_FIX) { nBA_init = TINY_FIX; }
    
    norxa = fx_div32(ad1_32_init, nBA_init); 
    norya = fx_div32(ad2_32_init, nBA_init); 
    norLa = fx_div32(det32_init, nBA_init);

    Dd[0] = xa + fx_mul(signo, fx_mul(rad, norxa));
    Dd[1] = ya + fx_mul(signo, fx_mul(rad, norya));
    Dd[2] = La + fx_mul(signo, fx_mul(rad, norLa));

    tray[0][0] = xa; tray[0][1] = ya; tray[0][2] = La;

    while ((ii < s.maxsteps) && (L > -256) && (cond1 == 0)) {
        x = Dd[0]; y = Dd[1]; L = Dd[2];

        if (ii == 1 && L < La) { 
            signo = -signo; 
            Dd[0] = xa + fx_mul(signo, fx_mul(rad, norxa)); 
            Dd[1] = ya + fx_mul(signo, fx_mul(rad, norya)); 
            Dd[2] = La + fx_mul(signo, fx_mul(rad, norLa));
            x = Dd[0]; y = Dd[1]; L = Dd[2];
        }

        trayC[0][0]=trayC[1][0]; trayC[0][1]=trayC[1][1]; trayC[0][2]=trayC[1][2];
        trayC[1][0]=trayC[2][0]; trayC[1][1]=trayC[2][1]; trayC[1][2]=trayC[2][2];
        trayC[2][0]=C1;          trayC[2][1]=C2;          trayC[2][2]=C3;

        tray[ii][0] = xa; tray[ii][1] = ya; tray[ii][2] = La;

        xa = x; ya = y; La = L;
        
        int i = 0; fx16 err = K_ONE;
        fx16 f[3], jacob[3][3], Jinv[3][3];

        while (i < 40) {
            calc_f_jacob_fix(&s, x, y, L, C1, C2, C3, rad, W_0, Q, f, jacob);
            err = fx_norm3d(f[0], f[1], f[2]);
            
            if (err <= s.paro) break;

            equilibrar_filas_matriz(jacob, f);
            
            invertir_3x3_fix(jacob, Jinv);
            fx32 d0 = (fx32)Jinv[0][0]*f[0] + (fx32)Jinv[0][1]*f[1] + (fx32)Jinv[0][2]*f[2];
            fx32 d1 = (fx32)Jinv[1][0]*f[0] + (fx32)Jinv[1][1]*f[1] + (fx32)Jinv[1][2]*f[2];
            fx32 d2 = (fx32)Jinv[2][0]*f[0] + (fx32)Jinv[2][1]*f[1] + (fx32)Jinv[2][2]*f[2];

            if (d0 > 25*512) { d0 = 25*512; } if (d0 < -25*512) { d0 = -25*512; }
            if (d1 > 25*512) { d1 = 25*512; } if (d1 < -25*512) { d1 = -25*512; }
            if (d2 > 25*512) { d2 = 25*512; } if (d2 < -25*512) { d2 = -25*512; }

            xa = x - (fx16)(d0 >> Q_SHIFT); ya = y - (fx16)(d1 >> Q_SHIFT); La = L - (fx16)(d2 >> Q_SHIFT);
            x = xa; y = ya; L = La;
            
            i++;
        }
        inr += i;
        
        if (ii > 3) {
            fx16 move_dist = fx_norm3d(tray[ii-1][0]-trayC[2][0], tray[ii-1][1]-trayC[2][1], tray[ii-1][2]-trayC[2][2]);
            if (move_dist > 5) { 
                int rever = 1, irot = 1, direction = 1, direc1 = 0;
                while ((irot < 3) && (rever == 1)) {
                    fx16 g1_0 = 2*(tray[ii-1][0]-trayC[2][0]); fx16 g1_1 = 2*(tray[ii-1][1]-trayC[2][1]); fx16 g1_2 = 2*(tray[ii-1][2]-trayC[2][2]);
                    fx16 n1 = fx_norm3d(g1_0, g1_1, g1_2); if (n1 < TINY_FIX) { n1 = TINY_FIX; }
                    fx16 t1_x = fx_acos(fx_div(g1_0, n1)); fx16 t1_y = fx_acos(fx_div(g1_1, n1));
                    
                    fx16 g2_0 = 2*(xa-C1); fx16 g2_1 = 2*(ya-C2); fx16 g2_2 = 2*(La-C3);
                    fx16 n2 = fx_norm3d(g2_0, g2_1, g2_2); if (n2 < TINY_FIX) { n2 = TINY_FIX; }
                    fx16 t2_x = fx_acos(fx_div(g2_0, n2)); fx16 t2_y = fx_acos(fx_div(g2_1, n2));
                    
                    rever = (fx_abs(t1_x - t2_x) < 5 && fx_abs(t1_y - t2_y) < 5) ? 1 : 0;
                    
                    if ((err > s.paro) || (rever == 1)) {
                        fx16 diff_C = fx_norm3d(trayC[2][0]-tray[ii-1][0], trayC[2][1]-tray[ii-1][1], trayC[2][2]-tray[ii-1][2]);
                        if (diff_C < TINY_FIX) { diff_C = TINY_FIX; }
                        fx16 arg_fi1 = fx_div(trayC[2][0]-tray[ii-1][0], diff_C); fx16 fi1 = fx_acos(arg_fi1);
                        
                        fx16 norxaux = fx_norm3d(Dd[0]-trayC[2][0], Dd[1]-trayC[2][1], Dd[2]-trayC[2][2]);
                        if (norxaux < TINY_FIX) { norxaux = TINY_FIX; }
                        fx16 term_aux = fx_mul(signo, fx_mul(rad, norxa));
                        fx16 val_aux = fx_div(term_aux, norxaux);
                        
                        if (irot == 1) { fx16 fi2 = fx_acos(val_aux); if (fi2 > fi1) { direc1 = 1; } else { direc1 = -1; } direction = direc1; } 
                        else { direction = -direction; }
                        
                        fx16 dx_rot = Dd[0] - C1; fx16 dy_rot = Dd[1] - C2; fx16 rx, ry; fx_rotacion(dx_rot, dy_rot, direction, &rx, &ry);
                        x = rx + C1; y = ry + C2; L = fx_div(y + fx_mul(s.m1, x) - (s.b0 + fx_mul(s.m1, s.a0)), term_L);
                        
                        i = 0; err = K_ONE;
                        while (i < 40) {
                            calc_f_jacob_fix(&s, x, y, L, C1, C2, C3, rad, W_0, Q, f, jacob);
                            err = fx_norm3d(f[0], f[1], f[2]);
                            
                            if (err <= s.paro) break;

                            equilibrar_filas_matriz(jacob, f);

                            invertir_3x3_fix(jacob, Jinv);
                            fx32 d0 = (fx32)Jinv[0][0]*f[0] + (fx32)Jinv[0][1]*f[1] + (fx32)Jinv[0][2]*f[2];
                            fx32 d1 = (fx32)Jinv[1][0]*f[0] + (fx32)Jinv[1][1]*f[1] + (fx32)Jinv[1][2]*f[2];
                            fx32 d2 = (fx32)Jinv[2][0]*f[0] + (fx32)Jinv[2][1]*f[1] + (fx32)Jinv[2][2]*f[2];

                            if (d0 > 25*512) { d0 = 25*512; } if (d0 < -25*512) { d0 = -25*512; }
                            if (d1 > 25*512) { d1 = 25*512; } if (d1 < -25*512) { d1 = -25*512; }
                            if (d2 > 25*512) { d2 = 25*512; } if (d2 < -25*512) { d2 = -25*512; }

                            xa = x - (fx16)(d0 >> Q_SHIFT); ya = y - (fx16)(d1 >> Q_SHIFT); La = L - (fx16)(d2 >> Q_SHIFT);
                            x = xa; y = ya; L = La;
                            
                            i++;
                        }
                        inr += i;
                    } irot++;
                }
            }
        }

        calc_f_jacob_fix(&s, xa, ya, La, C1, C2, C3, rad, W_0, Q, f, jacob);
        fx32 det32 = -((fx32)jacob[0][0]*jacob[1][1] - (fx32)jacob[0][1]*jacob[1][0]) >> Q_SHIFT;
        fx32 ad1_32 = ((fx32)jacob[0][2]*jacob[1][1] - (fx32)jacob[1][2]*jacob[0][1]) >> Q_SHIFT;
        fx32 ad2_32 = ((fx32)jacob[1][2]*jacob[0][0] - (fx32)jacob[0][2]*jacob[1][0]) >> Q_SHIFT;
        fx16 nBA = fx_norm3d((fx16)ad1_32, (fx16)ad2_32, (fx16)det32); 
        if (nBA < TINY_FIX) { nBA = TINY_FIX; }
        norxa = fx_div32(ad1_32, nBA); norya = fx_div32(ad2_32, nBA); norLa = fx_div32(det32, nBA);
        
        fx16 det = (fx16)det32; 
        fx16 div_exp = fx_abs(det); if (div_exp < TINY_FIX) { div_exp = TINY_FIX; }
        fx16 ratio = fx_div(fx_abs(m_L1), div_exp); fx16 term_exp = fx_exp_neg(ratio); 
        rad = fx_mul(s.rr, K_ONE + term_exp); m_L1 = det;
        
        fx16 norBA_corr = fx_norm3d(xa-tray[ii][0], ya-tray[ii][1], La-tray[ii][2]);
        if (norBA_corr < TINY_FIX) { norBA_corr = TINY_FIX; }
        if ( (r - rad) > fx_div(r, float_to_fix(3.0)) ) { rad = fx_div(r, float_to_fix(2.0)) + s.rr; }
        fx16 factor = fx_div(rad - r, norBA_corr);
        
        C1 = xa + fx_mul(factor, xa - tray[ii][0]); 
        C2 = ya + fx_mul(factor, ya - tray[ii][1]); 
        C3 = La + fx_mul(factor, La - tray[ii][2]);

        Dd[0] = C1 + fx_mul(signo, fx_mul(rad, norxa)); Dd[1] = C2 + fx_mul(signo, fx_mul(rad, norya)); Dd[2] = C3 + fx_mul(signo, fx_mul(rad, norLa));
        
        fx16 sx = (s.a1 > s.a0) ? K_ONE : -K_ONE; fx16 sy = (s.b1 > s.b0) ? K_ONE : -K_ONE;
        int lx = (sx == K_ONE) ? (xa >= s.a1 - 5) : (xa <= s.a1 + 5); 
        int ly = (sy == K_ONE) ? (ya >= s.b1 - 5) : (ya <= s.b1 + 5);
        if (La >= K_ONE && lx && ly) { cond1 = 1; }
        r = rad; ii++;
    }

    tray[ii][0] = xa;
    tray[ii][1] = ya;
    tray[ii][2] = La;

    printf("Pasos totales = %d\n", ii);
    printf("Iteraciones NR totales = %d\n", inr);

    FILE *fp2 = fopen("tray.txt", "w");
    if (!fp2) {
        printf("Error: No se pudo crear el archivo tray.txt\n");
        return 1;
    }
    
    char buf2[500];
    for (int j = 0; j <= ii; j++) {
        sprintf(buf2, "%f \t %f \n", fix_to_float(tray[j][0]), fix_to_float(tray[j][1]));
        fputs(buf2, fp2);
    }
    fclose(fp2);

    return 0;
}