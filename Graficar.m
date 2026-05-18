clc; clear all; close all;

port_name = "COM4"; 
baud_rate = 9600;   

try
    s = serialport(port_name, baud_rate);
    configureTerminator(s, "CR/LF");
    flush(s);
catch
    error('No se pudo abrir el puerto %s.', port_name);
end

disp('Puerto serial abierto con éxito. Preparando mapa...');
pause(1);

% X, Y, Radio, K
mapa_obstaculos = [
    0.154321	0.821098	0.045000	-0.06
0.432198	0.187654	0.052000	0.06
0.710543	0.334455	0.038000	0.06
0.284567	0.556789	0.061000	-0.06
0.654321	0.890123	0.040000	-0.06
0.890123	0.123456	0.047000	0.06
0.345678	0.765432	0.055000	-0.06
0.567890	0.678901	0.033000	-0.06
0.901234	0.445678	0.062000	0.06
0.190500	0.612000	0.041000	-0.06
0.410330	0.920100	0.050000	-0.06
0.730000	0.210400	0.035000	0.06
0.280110	0.150770	0.058000	0.06
0.850900	0.750800	0.042000	0.06
0.520440	0.110550	0.049000	0.06
0.180220	0.380330	0.037000	-0.06
0.620660	0.480770	0.054000	0.06
0.950880	0.650990	0.046000	0.06
0.380990	0.220110	0.039000	0.06
0.750110	0.850220	0.051000	-0.06
0.080330	0.720440	0.044000	-0.06
0.480550	0.820660	0.057000	-0.06
0.300770	0.300880	0.036000	0.06
0.220990	0.920110	0.048000	-0.06
0.580220	0.280330	0.053000	0.06
0.920440	0.220550	0.043000	0.06
0.450660	0.430770	0.038000	-0.06
0.680880	0.950990	0.034000	-0.06
0.110110	0.200220	0.046000	-0.06
0.740330	0.600440	0.059000	0.06

];

num_obstaculos = size(mapa_obstaculos, 1);
radio_robot = 0.03;

% ESCALAMIENTO A PUNTO FIJO Q7.9
obs_fix = zeros(num_obstaculos, 4);
for i = 1:num_obstaculos
    % s.mc[s.nc][0] = float_to_fix(v0);
    obs_x_fix = round(mapa_obstaculos(i, 1) * 512);
    
    % s.mc[s.nc][1] = float_to_fix(v1);
    obs_y_fix = round(mapa_obstaculos(i, 2) * 512);
    
    % s.mc[s.nc][2] = float_to_fix(v2);
    mrad_fix  = round(mapa_obstaculos(i, 3) * 512);
    
    % s.mc[s.nc][3] = float_to_fix(v3);
    k_amp_fix = round(mapa_obstaculos(i, 4) * 512);
    
    % Guardar para la trama UART
    obs_fix(i, :) = [obs_x_fix, obs_y_fix, mrad_fix, k_amp_fix];
end

% EMPAQUETADO DE DATOS (UART)
header = [hex2dec('AA'), hex2dec('55')];
to_bytes = @(val) [bitshift(typecast(int16(val), 'uint16'), -8), bitand(typecast(int16(val), 'uint16'), 255)];

bytes_nc = to_bytes(num_obstaculos);       
bytes_rr = to_bytes(10);        % 0.02 * 512
bytes_m1 = to_bytes(-2048);     % -4.0 * 512
bytes_m2 = to_bytes(-512);      % -1.0 * 512
bytes_ms = to_bytes(500);       % 500 (Entero)
bytes_a0 = to_bytes(0);         % 0.0 * 512
bytes_b0 = to_bytes(0);         % 0.0 * 512
bytes_a1 = to_bytes(512);       % 1.0 * 512
bytes_b1 = to_bytes(512);       % 1.0 * 512

bytes_obstaculos = [];
for i = 1:num_obstaculos
    bytes_obstaculos = [bytes_obstaculos, ...
                        to_bytes(obs_fix(i,1)), to_bytes(obs_fix(i,2)), ...
                        to_bytes(obs_fix(i,3)), to_bytes(obs_fix(i,4))];
end

% Trama final ensamblada
tx_data = [header, bytes_nc, bytes_rr, bytes_m1, bytes_m2, bytes_ms, ...
           bytes_a0, bytes_b0, bytes_a1, bytes_b1, bytes_obstaculos];

% Graficar
figure('Name', 'Trayectoria HPPM Multi-Obstáculo FPGA', 'NumberTitle', 'off');
hold on; grid on; axis equal;
xlim([-0.2 1.2]); ylim([-0.2 1.2]);
xlabel('Eje X'); ylabel('Eje Y');
title(sprintf('Esperando coordenadas... (Obstáculos: %d)', num_obstaculos));

plot(0, 0, 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Inicio');
plot(1, 1, 'r*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Meta');

% Dibujar todos los obstáculos
theta = linspace(0, 2*pi, 100);
for i = 1:num_obstaculos
    r_obs = mapa_obstaculos(i, 3);
    x_obs = mapa_obstaculos(i, 1);
    y_obs = mapa_obstaculos(i, 2);

    if mapa_obstaculos(i, 4) < 0
        color_linea = 'r-';  % Rojo (K negativo)
    else
        color_linea = 'm--'; % Magenta (K positivo)
    end
    
    if i == 1
        plot(x_obs + r_obs*cos(theta), y_obs + r_obs*sin(theta), color_linea, 'LineWidth', 1.5, 'DisplayName', 'Obstáculo');
    else
        plot(x_obs + r_obs*cos(theta), y_obs + r_obs*sin(theta), color_linea, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end
legend('Location', 'best');
drawnow;

% Tx / rx
disp('Transmitiendo mapa al FPGA...');
write(s, tx_data, "uint8");
disp('¡Mapa enviado! Escuchando al hardware...');
title('FPGA Calculando Trayectoria...');

puntos_recibidos = 0;
timeout_counter = 0;

while true
    if s.NumBytesAvailable >= 6
        rx_bytes = read(s, 6, "uint8");
        timeout_counter = 0; 
        
        x_int = double(rx_bytes(1))*256 + double(rx_bytes(2));
        if x_int >= 32768, x_int = x_int - 65536; end
        x_float = x_int / 512.0;
        
        y_int = double(rx_bytes(3))*256 + double(rx_bytes(4));
        if y_int >= 32768, y_int = y_int - 65536; end
        y_float = y_int / 512.0;
        
        l_int = double(rx_bytes(5))*256 + double(rx_bytes(6));
        if l_int >= 32768, l_int = l_int - 65536; end
        l_float = l_int / 512.0;
        
        plot(x_float, y_float, 'b.-', 'MarkerSize', 12, 'LineWidth', 1, 'HandleVisibility', 'off');
        drawnow;
        
        puntos_recibidos = puntos_recibidos + 1;
        fprintf('Paso %d | X: %.3f, Y: %.3f (L: %.3f)\n', puntos_recibidos, x_float, y_float, l_float);
        
        meta_x = 1.0; % sys_a1
        meta_y = 1.0; % sys_b1
        
        if l_float >= 0.995 && abs(x_float - meta_x) < 0.02 && abs(y_float - meta_y) < 0.02
            title('Trayectoria HPPM Completada Exitosamente');
            break;
        end
        
    else
        pause(0.1);
        timeout_counter = timeout_counter + 1;
        if timeout_counter > 20 && puntos_recibidos > 0
            title('Trayectoria Completada');
            break;
        end
    end
end

clear s;
disp('Puerto serial cerrado.'); 