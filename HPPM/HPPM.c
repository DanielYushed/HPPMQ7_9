#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define TRUE 1
#define FALSE 0

#define CLAMP_ACOS(x) ((x) > 1.0f ? 1.0f : ((x) < -1.0f ? -1.0f : (x)))

#define EPSILON 0.001f
#define EPSILON_SQ (EPSILON * EPSILON)
#define PI_F 3.141592653589793f

typedef struct {
    int num_obstaculos;                 
    float radio_esfera;               
    float pendiente_1, pendiente_2;           
    int max_pasos;           
    float inicio_x, inicio_y;           
    float meta_x, meta_y;           
    float **matriz_obs;             // [x, y, radio_cuadrado, k]
    float **historial_tray;           
    int pasos_totales;   
} SistemaHPPM;

typedef struct {
    float x, y, L;          
    float x_ant, y_ant, L_ant;       
    float c1, c2, c3;       
    float radio_actual, radio_ant;           
    
    float dir_x, dir_y, dir_L; 
    
    float jacobiano[3][3];      
    float jacobiano_inv[3][3];       
    float error_f[3][1];          
    float paso_pred[3][1];          
    float paso_corr[3][1];        
    
    float energia_meta;                
    float energia_inicio;              
    int iter_paso;                 
    int iter_newton;                
    float det_ant;             
} EstadoHPPM;

static inline float f_potencial(float x, float y) {
    return 1.0f; 
}

float** crear_matriz(int filas, int cols) {
    float **m = (float**)malloc(filas * sizeof(float*));
    float *data = (float*)calloc(filas * cols, sizeof(float));
    if (!m || !data) exit(1);
    for(int i = 0; i < filas; i++) {
        m[i] = data + i * cols;
    }
    return m;
}

void liberar_matriz(float **m) {
    if (!m) return;
    free(m[0]); 
    free(m);    
}

void invertir_matriz_3x3(float J[3][3], float Ji[3][3]) {
    float c00 = J[1][1]*J[2][2] - J[1][2]*J[2][1];
    float c01 = J[1][0]*J[2][2] - J[1][2]*J[2][0];
    float c02 = J[1][0]*J[2][1] - J[1][1]*J[2][0];

    float det = J[0][0]*c00 - J[0][1]*c01 + J[0][2]*c02;
    float invDet = (fabsf(det) < 1e-9f) ? 0.0f : 1.0f / det;

    Ji[0][0] =  c00 * invDet;
    Ji[1][0] = -c01 * invDet;
    Ji[2][0] =  c02 * invDet;
    Ji[0][1] = -(J[0][1]*J[2][2] - J[0][2]*J[2][1]) * invDet;
    Ji[1][1] =  (J[0][0]*J[2][2] - J[0][2]*J[2][0]) * invDet;
    Ji[2][1] = -(J[0][0]*J[2][1] - J[0][1]*J[2][0]) * invDet;
    Ji[0][2] =  (J[0][1]*J[1][2] - J[0][2]*J[1][1]) * invDet;
    Ji[1][2] = -(J[0][0]*J[1][2] - J[0][2]*J[1][0]) * invDet;
    Ji[2][2] =  (J[0][0]*J[1][1] - J[0][1]*J[1][0]) * invDet;
}

void actualizar_jacobiano_y_f(SistemaHPPM *s, EstadoHPPM *e, int calc_f) {
    float W_px = 0.0f, W_py = 0.0f, W = 0.0f, aux_Wp = 0.0f;
    float Obs, raiz_Obs, abs_1, D, dD_dObs;
    float dx, dy;

    float fP_val = f_potencial(e->x, e->y);

    for(int k = 0; k < s->num_obstaculos; k++) {
        dx = e->x - s->matriz_obs[k][0];
        dy = e->y - s->matriz_obs[k][1];
        
        Obs = (dx * dx) + (dy * dy) - s->matriz_obs[k][2]; 

        raiz_Obs = sqrtf((Obs * Obs) + EPSILON_SQ); 
        abs_1 = raiz_Obs - EPSILON;
        
        D = Obs + abs_1; 
        if(fabsf(D) < 1e-12f) D = 1e-12f;

        dD_dObs = 1.0f + (Obs / raiz_Obs); 

        float P_bar = s->matriz_obs[k][3]; 
        aux_Wp = -(P_bar * fP_val) / (D * D) * dD_dObs;

        W_px += (aux_Wp * (2.0f * dx)); 
        W_py += (aux_Wp * (2.0f * dy));
        
        if(calc_f) {
            W += (P_bar * fP_val) / D;
        }
    }

    e->jacobiano[0][0] = -s->pendiente_1; 
    e->jacobiano[0][1] = -1.0f; 
    e->jacobiano[0][2] = -s->inicio_y - s->pendiente_1 * s->inicio_x + s->pendiente_1 * s->meta_x + s->meta_y;

    e->jacobiano[1][0] = -s->pendiente_2 + W_px; 
    e->jacobiano[1][1] = -1.0f + W_py;
    e->jacobiano[1][2] = -s->inicio_y - s->pendiente_2*s->inicio_x + s->meta_y + s->pendiente_2*s->meta_x + e->energia_inicio - e->energia_meta;
    
    e->jacobiano[2][0] = 2.0f * (e->x - e->c1); 
    e->jacobiano[2][1] = 2.0f * (e->y - e->c2);
    e->jacobiano[2][2] = 2.0f * (e->L - e->c3);

    if (calc_f) {
        float const_H1 = -s->inicio_y - s->pendiente_1*s->inicio_x + s->pendiente_1*s->meta_x + s->meta_y;
        float const_H2 = -s->inicio_y - s->pendiente_2*s->inicio_x + (s->meta_y + s->pendiente_2*s->meta_x) + e->energia_inicio - e->energia_meta;
        
        e->error_f[0][0] = -e->y - s->pendiente_1*e->x + s->pendiente_1*s->meta_x + s->meta_y - (1.0f - e->L)*const_H1;
        e->error_f[1][0] = (-e->y - s->pendiente_2*e->x + (s->meta_y + s->pendiente_2*s->meta_x) + W - e->energia_meta) - (1.0f - e->L)*const_H2;
        
        float dCx = e->x - e->c1;
        float dCy = e->y - e->c2;
        float dCL = e->L - e->c3;
        e->error_f[2][0] = (dCx * dCx) + (dCy * dCy) + (dCL * dCL) - (e->radio_actual * e->radio_actual);
    }
}

void leer_parametros(SistemaHPPM *s) {
    FILE *f1 = fopen("pendientes.txt", "r");
    if (f1 == NULL) { printf("Error: No se pudo abrir pendientes.txt\n"); exit(1); }

    fscanf(f1, "%f", &s->radio_esfera);
    fscanf(f1, "%f", &s->pendiente_1);
    fscanf(f1, "%f", &s->pendiente_2);
    fscanf(f1, "%d", &s->max_pasos);
    
    fscanf(f1, "%f", &s->inicio_x);
    fscanf(f1, "%f", &s->inicio_y);
    fscanf(f1, "%f", &s->meta_x);
    fscanf(f1, "%f", &s->meta_y);
    fclose(f1);

    FILE *f2 = fopen("obstaculos1.txt", "r");
    if (f2 == NULL) { printf("Error: No se pudo abrir obstaculos1.txt\n"); exit(1); }
    
    s->num_obstaculos = 0;
    float v0, v1, v2, v3;
    
    while (fscanf(f2, "%f %f %f %f", &v0, &v1, &v2, &v3) == 4) { 
        s->num_obstaculos++;
    }
    rewind(f2);
    
    s->matriz_obs = crear_matriz(s->num_obstaculos, 4);
    
    for (int k = 0; k < s->num_obstaculos; k++){
        if (fscanf(f2, "%f %f %f %f", &v0, &v1, &v2, &v3) == 4) {
            s->matriz_obs[k][0] = v0;
            s->matriz_obs[k][1] = v1;
            s->matriz_obs[k][2] = v2 * v2;
            s->matriz_obs[k][3] = v3;
        } else {
            s->matriz_obs[k][0] = 0.0f; s->matriz_obs[k][1] = 0.0f; s->matriz_obs[k][2] = 0.0f; s->matriz_obs[k][3] = 0.0f;
        }
    }
    fclose(f2);

    s->historial_tray = crear_matriz(s->max_pasos, 3);
    
    printf("Datos leidos: %d circulos detectados automaticamente.\n", s->num_obstaculos);
    printf("Inicio: %f, %f -> Meta: %f, %f\n", s->inicio_x, s->inicio_y, s->meta_x, s->meta_y);
}

void calcular_trayectoria(SistemaHPPM *s) {
    EstadoHPPM e; 
    memset(&e, 0, sizeof(EstadoHPPM));
    
    e.det_ant = 1.0f; 
    e.iter_paso = 1;
    e.radio_actual = s->radio_esfera; 
    e.radio_ant = s->radio_esfera;

    e.x = s->inicio_x; 
    e.y = s->inicio_y;
    e.x_ant = s->inicio_x; 
    e.y_ant = s->inicio_y;
    
    float term_L = (s->meta_y + (s->pendiente_1 * (s->meta_x - s->inicio_x)) - s->inicio_y);
    if (fabsf(term_L) < 1e-5f) term_L = 1e-5f;

    e.L_ant = (e.y_ant + (s->pendiente_1 * e.x_ant) - (s->inicio_y + s->pendiente_1 * s->inicio_x)) / term_L; 
    e.L = e.L_ant;
    e.c1 = e.x_ant; e.c2 = e.y_ant; e.c3 = e.L_ant; 
    
    float paro = 0.000001f; 
    float paro_sq = paro * paro; 
    int maxiter = 40; 
    int signo = (s->pendiente_1 > s->pendiente_2) ? -1 : 1; 
    int sig_x = (s->meta_x > s->inicio_x) ? 1 : -1;
    int sig_y = (s->meta_y > s->inicio_y) ? 1 : -1;
    float trayC[3][3] = {0}; 
    int cond1 = 0;

    s->historial_tray[0][0] = e.x_ant;
    s->historial_tray[0][1] = e.y_ant;
    s->historial_tray[0][2] = e.L_ant;

    float fP_meta = f_potencial(s->meta_x, s->meta_y);
    float fP_inicio = f_potencial(s->inicio_x, s->inicio_y);

    for(int k = 0; k < s->num_obstaculos; k++) {
        float P_bar = s->matriz_obs[k][3];
        float r_sq = s->matriz_obs[k][2];

        float dx_Q = s->meta_x - s->matriz_obs[k][0];
        float dy_Q = s->meta_y - s->matriz_obs[k][1];
        float Obs_Q = (dx_Q * dx_Q) + (dy_Q * dy_Q) - r_sq;
        float raiz_Q = sqrtf((Obs_Q * Obs_Q) + EPSILON_SQ);
        float D_Q = Obs_Q + (raiz_Q - EPSILON);
        if (fabsf(D_Q) < 1e-12f) D_Q = 1e-12f;
        e.energia_meta += (P_bar * fP_meta) / D_Q;

        float dx_W = s->inicio_x - s->matriz_obs[k][0];
        float dy_W = s->inicio_y - s->matriz_obs[k][1];
        float Obs_W = (dx_W * dx_W) + (dy_W * dy_W) - r_sq;
        float raiz_W = sqrtf((Obs_W * Obs_W) + EPSILON_SQ);
        float D_W = Obs_W + (raiz_W - EPSILON);
        if (fabsf(D_W) < 1e-12f) D_W = 1e-12f;
        e.energia_inicio += (P_bar * fP_inicio) / D_W;
    }

    actualizar_jacobiano_y_f(s, &e, FALSE);
    
    float aux1 = e.jacobiano[0][2]*e.jacobiano[1][1] - e.jacobiano[1][2]*e.jacobiano[0][1];
    float aux2 = e.jacobiano[1][2]*e.jacobiano[0][0] - e.jacobiano[0][2]*e.jacobiano[1][0];
    float det0 = -(e.jacobiano[0][0]*e.jacobiano[1][1] - e.jacobiano[0][1]*e.jacobiano[1][0]);
    float norBA0 = sqrtf((aux1*aux1) + (aux2*aux2) + (det0*det0));
    
    if (norBA0 < 1e-9f) norBA0 = 1e-9f;
    
    e.dir_x = aux1/norBA0; 
    e.dir_y = aux2/norBA0; 
    e.dir_L = det0/norBA0;

    e.paso_pred[0][0] = e.x_ant + signo*(s->radio_esfera * e.dir_x);
    e.paso_pred[1][0] = e.y_ant + signo*(s->radio_esfera * e.dir_y);
    e.paso_pred[2][0] = e.L_ant + signo*(s->radio_esfera * e.dir_L);
    e.det_ant = det0; 

    while ((e.iter_paso < s->max_pasos-2) && (e.L > -0.5f) && (cond1 == 0)) {
        e.x = e.paso_pred[0][0]; e.y = e.paso_pred[1][0]; e.L = e.paso_pred[2][0];
        
        memcpy(&trayC[0][0], &trayC[1][0], 3 * sizeof(float));
        memcpy(&trayC[1][0], &trayC[2][0], 3 * sizeof(float));
        trayC[2][0] = e.c1; trayC[2][1] = e.c2; trayC[2][2] = e.c3;

        s->historial_tray[e.iter_paso][0] = e.x_ant; 
        s->historial_tray[e.iter_paso][1] = e.y_ant; 
        s->historial_tray[e.iter_paso][2] = e.L_ant;

        int i = 0; 
        float err_sq = 1.0f;
        
        while (i < maxiter) {
            actualizar_jacobiano_y_f(s, &e, TRUE);
            err_sq = (e.error_f[0][0]*e.error_f[0][0]) + (e.error_f[1][0]*e.error_f[1][0]) + (e.error_f[2][0]*e.error_f[2][0]);
            if (err_sq <= paro_sq) break;

            invertir_matriz_3x3(e.jacobiano, e.jacobiano_inv);
            
            float a1 = (e.jacobiano_inv[0][0]*e.error_f[0][0] + e.jacobiano_inv[0][1]*e.error_f[1][0] + e.jacobiano_inv[0][2]*e.error_f[2][0]);
            float a2 = (e.jacobiano_inv[1][0]*e.error_f[0][0] + e.jacobiano_inv[1][1]*e.error_f[1][0] + e.jacobiano_inv[1][2]*e.error_f[2][0]);
            float a3 = (e.jacobiano_inv[2][0]*e.error_f[0][0] + e.jacobiano_inv[2][1]*e.error_f[1][0] + e.jacobiano_inv[2][2]*e.error_f[2][0]);
            
            e.x -= a1; 
            e.y -= a2; 
            e.L -= a3;
            e.x_ant = e.x; 
            e.y_ant = e.y; 
            e.L_ant = e.L;
            i++;
        }
        e.iter_newton += i;

        // Antireversion 
        if (e.iter_paso > 3) {
            int rever = 1, irot = 1, direction = 1;
            while ((irot < 3) && (rever == 1)) {
                float gr[3]; 
                
                gr[0] = 2.0f * (s->historial_tray[e.iter_paso-1][0] - trayC[2][0]);
                gr[1] = 2.0f * (s->historial_tray[e.iter_paso-1][1] - trayC[2][1]);
                gr[2] = 2.0f * (s->historial_tray[e.iter_paso-1][2] - trayC[2][2]);
                float nor_gr = sqrtf(gr[0]*gr[0] + gr[1]*gr[1] + gr[2]*gr[2]);
                if (nor_gr < 1e-9f) nor_gr = 1e-9f;
                
                float t1[3], t2[3];
                for(int d=0; d<3; d++) t1[d] = floorf((acosf(CLAMP_ACOS(gr[d]/nor_gr))*(180.0f/PI_F))*100.0f)/100.0f;
                
                e.x = e.x_ant; e.y = e.y_ant; e.L = e.L_ant;
                gr[0] = 2.0f * (e.x - trayC[2][0]); 
                gr[1] = 2.0f * (e.y - trayC[2][1]); 
                gr[2] = 2.0f * (e.L - trayC[2][2]);
                nor_gr = sqrtf(gr[0]*gr[0] + gr[1]*gr[1] + gr[2]*gr[2]);
                if (nor_gr < 1e-9f) nor_gr = 1e-9f;

                for(int d=0; d<3; d++) t2[d] = floorf((acosf(CLAMP_ACOS(gr[d]/nor_gr))*(180.0f/PI_F))*100.0f)/100.0f;
                
                rever = (t1[0] == t2[0] && t1[1] == t2[1]) ? 1 : 0;
                
                if (err_sq > paro_sq || rever == 1) {
                    float dx_fi = trayC[2][0]-s->historial_tray[e.iter_paso-1][0];
                    float dy_fi = trayC[2][1]-s->historial_tray[e.iter_paso-1][1];
                    float dL_fi = trayC[2][2]-s->historial_tray[e.iter_paso-1][2];
                    
                    float norfi = sqrtf(dx_fi*dx_fi + dy_fi*dy_fi + dL_fi*dL_fi);
                    if(norfi < 1e-9f) norfi = 1e-9f;
                    float fi1 = floorf((acosf(CLAMP_ACOS(dx_fi/norfi))*(180.0f/PI_F))*100.0f)/100.0f;
                    
                    float dx_aux = e.paso_pred[0][0]-trayC[2][0];
                    float dy_aux = e.paso_pred[1][0]-trayC[2][1];
                    float dL_aux = e.paso_pred[2][0]-trayC[2][2];
                    
                    float norxaux = sqrtf(dx_aux*dx_aux + dy_aux*dy_aux + dL_aux*dL_aux);
                    if(norxaux < 1e-9f) norxaux = 1e-9f;
                    
                    actualizar_jacobiano_y_f(s, &e, FALSE); 
                    norxaux = (signo*(e.radio_actual*e.dir_x))/norxaux; 
                    
                    if (irot == 1) {
                        float fi2 = floorf((acosf(CLAMP_ACOS(norxaux))*(180.0f/PI_F))*100.0f)/100.0f;
                        direction = (fi2 > fi1) ? 1 : -1;
                    } else {
                        direction *= -1;
                    }

                    float ang = (PI_F / 4.0f) * direction;
                    float cos_a = cosf(ang);
                    float sin_a = sinf(ang);
                    
                    float Pnp[2] = {e.paso_pred[0][0]-trayC[2][0], e.paso_pred[1][0]-trayC[2][1]};
                    float Prot[2] = {cos_a*Pnp[0] - sin_a*Pnp[1], sin_a*Pnp[0] + cos_a*Pnp[1]};
                    
                    e.paso_corr[0][0] = Prot[0]+trayC[2][0]; 
                    e.paso_corr[1][0] = Prot[1]+trayC[2][1];
                    e.paso_corr[2][0] = (e.paso_corr[1][0]+(s->pendiente_1*e.paso_corr[0][0])-(s->inicio_y+s->pendiente_1*s->inicio_x))/term_L;

                    e.x = e.paso_corr[0][0]; e.y = e.paso_corr[1][0]; e.L = e.paso_corr[2][0];
                    i = 0; 
                    err_sq = 1.0f;
                    
                    while (i < maxiter) {
                        actualizar_jacobiano_y_f(s, &e, TRUE);
                        err_sq = (e.error_f[0][0]*e.error_f[0][0]) + (e.error_f[1][0]*e.error_f[1][0]) + (e.error_f[2][0]*e.error_f[2][0]);
                        if (err_sq <= paro_sq) break;

                        invertir_matriz_3x3(e.jacobiano, e.jacobiano_inv);
                        float ra1=(e.jacobiano_inv[0][0]*e.error_f[0][0]+e.jacobiano_inv[0][1]*e.error_f[1][0]+e.jacobiano_inv[0][2]*e.error_f[2][0]);
                        float ra2=(e.jacobiano_inv[1][0]*e.error_f[0][0]+e.jacobiano_inv[1][1]*e.error_f[1][0]+e.jacobiano_inv[1][2]*e.error_f[2][0]);
                        float ra3=(e.jacobiano_inv[2][0]*e.error_f[0][0]+e.jacobiano_inv[2][1]*e.error_f[1][0]+e.jacobiano_inv[2][2]*e.error_f[2][0]);
                        
                        e.x -= ra1; 
                        e.y -= ra2; 
                        e.L -= ra3; 
                        e.x_ant = e.x; 
                        e.y_ant = e.y; 
                        e.L_ant = e.L;
                        i++;
                    }
                    e.iter_newton += i;
                }
            }
            irot++; 
        }

        actualizar_jacobiano_y_f(s, &e, FALSE);
        float det = -1.0f*(e.jacobiano[0][0]*e.jacobiano[1][1] - e.jacobiano[0][1]*e.jacobiano[1][0]);
        float den = e.jacobiano[0][0]*e.jacobiano[1][1] - e.jacobiano[0][1]*e.jacobiano[1][0];
        if(fabsf(den) < 1e-9f) den = 1e-9f; 

        float aux_Dd1 = (-det*e.jacobiano[0][2])*(e.jacobiano[1][1]/den) + (-det*e.jacobiano[1][2])*(-e.jacobiano[0][1]/den);
        float aux_Dd2 = (-det*e.jacobiano[0][2])*(-e.jacobiano[1][0]/den) + (-det*e.jacobiano[1][2])*(e.jacobiano[0][0]/den);
        float aux_Dd3 = det;
        
        float norB_A = sqrtf((aux_Dd1*aux_Dd1) + (aux_Dd2*aux_Dd2) + (aux_Dd3*aux_Dd3));
        if(norB_A < 1e-9f) norB_A = 1e-9f;
        
        e.dir_x = aux_Dd1 / norB_A; 
        e.dir_y = aux_Dd2 / norB_A; 
        e.dir_L = aux_Dd3 / norB_A;
        
        float m_L2 = det;
        float exp_term = expf(-fabsf(e.det_ant) / sqrtf((m_L2*m_L2) + 1e-9f));
        e.radio_actual = s->radio_esfera * (1.0f + exp_term); 
        e.det_ant = m_L2;
        
        if ((e.radio_ant - e.radio_actual) > (e.radio_ant / 3.0f)) {
            e.radio_actual = (e.radio_ant / 2.0f) + s->radio_esfera;
        }
        
        float dx_BA = e.x_ant - s->historial_tray[e.iter_paso][0];
        float dy_BA = e.y_ant - s->historial_tray[e.iter_paso][1];
        float dL_BA = e.L_ant - s->historial_tray[e.iter_paso][2];
        float norBA = sqrtf((dx_BA*dx_BA) + (dy_BA*dy_BA) + (dL_BA*dL_BA));
        
        if(fabsf(norBA) < 1e-9f) norBA = 1e-9f;

        float factor = (e.radio_actual - e.radio_ant) / norBA;
        e.c1 = e.x_ant + factor*(dx_BA);
        e.c2 = e.y_ant + factor*(dy_BA);
        e.c3 = e.L_ant + factor*(dL_BA);
        
        e.paso_pred[0][0] = e.c1 + signo*(e.radio_actual*e.dir_x);
        e.paso_pred[1][0] = e.c2 + signo*(e.radio_actual*e.dir_y);
        e.paso_pred[2][0] = e.c3 + signo*(e.radio_actual*e.dir_L);

        int lim_x = (sig_x==1) ? (e.x_ant < s->meta_x-0.01f ? 0 : 1) : (e.x_ant > s->meta_x+0.01f ? 0 : 1);
        int lim_y = (sig_y==1) ? (e.y_ant < s->meta_y-0.01f ? 0 : 1) : (e.y_ant > s->meta_y+0.01f ? 0 : 1);
        cond1 = (e.L_ant >= 1 && lim_x && lim_y) ? 1 : 0;

        e.radio_ant = e.radio_actual;
        e.iter_paso++;
    }

    s->historial_tray[e.iter_paso][0] = e.x_ant;
    s->historial_tray[e.iter_paso][1] = e.y_ant;
    s->historial_tray[e.iter_paso][2] = e.L_ant;
    s->pasos_totales = e.iter_paso + 1; 

    printf("Pasos totales: %d\n", e.iter_paso+1);
}  

void guardar_resultados(SistemaHPPM *s) {
    FILE *fp2 = fopen("tray.txt", "w");
    if (fp2 == NULL) {
        printf("Error: No se pudo crear tray.txt\n");
        return;
    }
    for (int i = 0; i < s->pasos_totales; i++){
        fprintf(fp2, "%f \t %f \n", s->historial_tray[i][0], s->historial_tray[i][1]);
    }
    fclose(fp2);
    printf("tray.txt generado exitosamente.\n");
}

int main() {
    SistemaHPPM s;
    memset(&s, 0, sizeof(SistemaHPPM));

    leer_parametros(&s);
    calcular_trayectoria(&s); 
    guardar_resultados(&s);
    
    liberar_matriz(s.matriz_obs);
    liberar_matriz(s.historial_tray);

    return 0;
} 