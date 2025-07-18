%% Datos iniciales
clc; clear; close all;
m = 0.01165; % kg
m_adicional = 0.028 - m; % kg
l = 0.145; % m
g = 9.81; % m/s^2
I = (1/3*m + m_adicional)*l^2; % usando ejes paralelos + masa puntual
I = I * 85;
punto_operacion = 40; % grados
k = (m*g*l/(2*I) + m_adicional * g * l / I);

%% Experimento de PWM vs Fuerza del motor
K_pwm_F = 0.0002; % pendiente de la curva fuerza del motor vs pwm (%) (para el caso del brazo con masa adicional)

%% Experimento fricción (dejar caer el brazo)
alpha_experimental = 2.51e-2; % obtenido de los promedios del experimento de dejar caer el brazo
B = 2*I*alpha_experimental; % hallado sabiendo que alpha = termino_de_s / 2

%% Conversiones
K_rad_grados = 180 / pi;

%% 1) Carga los datos experimentales desde Excel de los experimentos
% Experimento fricción
M = readmatrix('datos_caida_brazo_exp_1.xlsx','Sheet','Hoja1');
t_exp     = M(:,1);      % columna de tiempos (s)
theta_exp = M(:,2);      % columna de ángulos (grados)
angulo_inicial = theta_exp(1) * pi / 180; % angulo inicial de la prueba, en radianes 

%% 2) Lanza la simulación de Simulink
simOut  = sim('planta','ReturnWorkspaceOutputs','on');

%% 3) Extrae la señal simulada de la variable 'out'
ts  = simOut.get('simout');% timeseries
t_sim    = ts.time;        % tiempo de simulación
theta_sim = ts.data;       % ángulo simulado (grados)

%% 4) Grafica ambas curvas superpuestas
figure; hold on;
plot(t_exp,     theta_exp, 'ro',   'MarkerSize',6, 'DisplayName','Datos Exp');
plot(t_sim,     theta_sim, 'b-',   'LineWidth',1.5,'DisplayName','Simulink');
xlabel('Tiempo (s)');
ylabel('Ángulo \theta (°)');
title ('Comparación: Datos Excel vs. Salida Simulink');
legend('Location','best');
grid on;

%% 6) Comparación Simulink vs Real desde 'diagrama_pwm_vs_fuerza.xlsx'
% Lee la hoja "Simulink"
M_sim   = readmatrix('diagrama_pwm_vs_fuerza.xlsx', 'Sheet','Simulink');
PWM_sim = M_sim(:,1);   % Columna A: PWM
F_sim   = M_sim(:,2);   % Columna B: F_motor

% Lee la hoja "Real"
M_real   = readmatrix('diagrama_pwm_vs_fuerza.xlsx', 'Sheet','Real');
PWM_real = M_real(:,1);
F_real   = M_real(:,2);

% Grafica comparación
figure; hold on;
plot(PWM_sim, F_sim,   'b-o', 'LineWidth',1.5, 'DisplayName','Simulink');
plot(PWM_real, F_real, 'r-s', 'LineWidth',1.5, 'DisplayName','Real');
xlabel('PWM (counts)');
ylabel('F\_motor (N)');
title('Comparación Fuerza Motor: Simulink vs Real');
legend('Location','best');
grid on;

%% Función de transferencia lazo cerrado para controlador PID y sin red.
s = tf('s');

Gplanta = (l*K_pwm_F)/(I*(s^2+B/I*s+k*cosd(punto_operacion)));

[num, den] = tfdata(Gplanta, 'v');

%% % Controlador PD I
syms s Kp Kd real

% Planta simbólica
G_sym = (l/I) / (s^2 + (B/I)*s + Kgrav);

% Controlador PD
C_sym = Kp + Kd * s;

% Lazo abierto simbólico
Lazo_abierto = C_sym * G_sym;

T_sym = simplify( C_sym * G_sym / (1 + C_sym * G_sym) );

disp('--- Función de transferencia en lazo cerrado (C*G)/(1 + C*G) ---');
pretty(T_sym)

% Extraer solo el denominador
[~, Den_T] = numden(T_sym);
Den_T = expand(Den_T); % Opcional: expandir para mayor claridad

% Mostrar el denominador
disp('--- Denominador de la función de transferencia en lazo cerrado ---');
pretty(Den_T)

Q = coeffs(Den_T, Kd);
if length(Q) == 2
    Q = Q(2);  % El segundo término corresponde al coeficiente de Kd
else
    Q = 0; % Por si no hay término con Kd
end

% Mostrar resultados
disp('--- Término que acompaña a Kd (Q) ---');
pretty(Q)

P = simplify(Den_T - Kd * Q);

disp('--- Parte independiente de Kd (P) ---');
pretty(P)



% Parámetros del controlador
Kp = 10;
Kd = 3.19;
Ki = 0.01;

% Controlador PID y planta G ya definidos antes
C = (Kp + Kd*s + Ki/s); % Controlador PID
Go = feedback(C * Gplanta, 1); % Lazo abierto
figure; hold on;
step(Go);
title('Respuesta al escalón – Lazo cerrado con PID');
grid on;


figure; hold on;
bode(Go);
margin(Go);
title('Bode y márgenes – Lazo cerrado con PID');
grid on;

% %% Incertidumbres
% theta_abs = abs(theta_exp);
% [amp_pk, idx_pk] = findpeaks(theta_abs, ...
%                              'MinPeakProminence',5, ...
%                              'MinPeakDistance',50);
% 
% n = numel(amp_pk);
% if n < 2
%     error('Necesitas al menos dos picos positivos para estimar α.');
% end
% 
% t_pk = t_exp(idx_pk);
% y    = log(amp_pk);
% 
% % Ajuste lineal manual -> coef b = pendiente
% coeffs = polyfit(t_pk, y, 1);    % y = a + b t
% b      = coeffs(1);              % pendiente (negativa)
% alpha  = -b;
% 
% % --- error estándar de la pendiente -------------------------------
% yfit = polyval(coeffs, t_pk);
% SE2  = sum( (y - yfit).^2 ) / (n - 2);      % varianza residual
% Sxx  = sum( (t_pk - mean(t_pk)).^2 );       % ∑(t-mean)^2
% u_alpha = sqrt(SE2 / Sxx);                  % desviación típica de b
%                                             % (ya con signo negativo quitado)
% 
% % ----------- ACTUALIZA B Y SU INCERTIDUMBRE ------------------------
% B = 2*I*alpha;
% 
% % Resoluciones → incertidumbres tipo-B
% u_m   = 0.001 /(2*sqrt(3));          % báscula 1 g   → 2.89e-4 kg
% u_mad = u_m;                         % misma báscula
% u_l   = 0.00002 /(2*sqrt(3));        % pie de rey 0.02 mm → 5.77e-6 m
% 
% % Derivadas parciales de I = (1/3 m + m_ad) l²
% dIdm    = (1/3)*l^2;
% dIdmad  =        l^2;
% dIdl    = 2*l*(1/3*m + m_adicional);
% 
% % Incertidumbre combinada de I
% u_I = sqrt( (dIdm   *u_m  )^2 + ...
%             (dIdmad *u_mad)^2 + ...
%             (dIdl   *u_l  )^2 );
% 
% 
% rel_uI     = u_I/I;
% rel_uAlpha = u_alpha/alpha;
% u_B        = B*sqrt(rel_uI^2 + rel_uAlpha^2);
% 
% fprintf('\n--- INCERTIDUMBRES (1·σ) --------------------------------\n');
% fprintf('n picos = %d\n', n)
% fprintf('I  = %.4e  ± %.2e   (rel %.3f %% )\n', I, u_I, 100*rel_uI);
% fprintf('α  = %.5f  ± %.2e   1/s   (rel %.3f %% )\n', ...
%         alpha, u_alpha, 100*rel_uAlpha);
% fprintf('B  = %.3e  ± %.2e  N·m·s  (rel %.3f %% )\n\n', ...
%         B, u_B, 100*u_B/B);
% 
% 
% % INCERTIDUMBRES (relativas, 1·σ → conviene pasarlas a ± %)
% % --- porcentajes ±% basados en las incertidumbres calculadas ----------
% pct_I = 100 * u_I / I;        % ← 0.018 % automáticamente
% pct_B = 100 * u_B / B;        % ← 1.26  % automáticamente
% pct_L = 100 * u_l / l;        % ← 0.004 %
% pct_m = 100 * u_m / m;        % ← 2.48 %
% pct_g = 0.01;                 % gravedad: se asume 0.01 %
% 
% m_u   = ureal('m' , 0.01165 , 'Percentage', pct_m);
% mad_u = ureal('mad', 0.01735 , 'Percentage', pct_m);
% g_u   = ureal('g' , 9.81    , 'Percentage', pct_g);
% L_u   = ureal('L' , 0.145   , 'Percentage', pct_L);
% I_u   = ureal('I' , I       , 'Percentage', pct_I);   % I nominal calculado
% B_u   = ureal('B' , B       , 'Percentage', pct_B);   % B nominal recalculado
% 
% thStar = deg2rad(80);
% 
% %% --- 2. Planta nominal y planta incierta --------------------------------
% s = tf('s');
% Kgrav_nom = ((m*g*l + 2*m_adicional*g*l)/(2*I))*cos(thStar);
% G_nom     = l / ( I*(s^2 + (B/I)*s + Kgrav_nom) );
% 
% Kgrav_unc = ((m_u*g_u*L_u + 2*mad_u*g_u*L_u)/(2*I_u))*cos(thStar);
% G_unc     = L_u/I_u / ( s^2 + (B_u/I_u)*s + Kgrav_unc );
% 
% %% --- 3. Compensador total (PID + lag) ----------------------------------
% C_pid  = Kp + Kd*s + Ki/s;
% 
% L_nom  = C_pid * G_nom;               % lazo abierto nominal
% L_unc  = C_pid * G_unc;               % lazo abierto incierto
% 
% %% --- 4. Márgenes nominales ---------------------------------------------
% [GM_nom, PM_nom] = margin(L_nom);
% fprintf('GM nominal = %.2f dB | PM nominal = %.1f°\n', ...
%         20*log10(GM_nom), PM_nom);
% 
% %% --- 5. Márgenes peor caso (Monte-Carlo) --------------------------------
% rng default
% Ns   = 300;
% GMmin =  inf;  PMmin =  inf;
% 
% Lsamp = usample(L_unc, Ns);     % 300 plantas → arreglo LTI
% 
% for k = 1:Ns
%     Lk = Lsamp(:,:,k);          % ← toma la k-ésima planta
%     [gm, pm] = margin(Lk);      % márgenes de esa planta
%     GMmin = min(GMmin, 20*log10(gm));
%     PMmin = min(PMmin, pm);
% end
% 
% fprintf('GM peor caso (MC, %d muestras) = %.2f dB\n', Ns, GMmin);
% fprintf('PM peor caso (MC, %d muestras) = %.1f°\n',  Ns, PMmin);
% 
% %% --- 6. (Opcional) Robust CT: rangos exactos ----------------------------
% % Si tienes Robust Control Toolbox descomenta:
% % M = allmargin(L_unc);
% % disp('Rango GM [dB]'); disp(20*log10(M.GM));
% % disp('Rango PM [°]');  disp(M.PM);
% % [wcGM, wcPM] = wcgain(L_unc);
% % fprintf('GM worst-case (analítico) = %.2f dB\n', 20*log10(wcGM.GainMargin(1)));
% % fprintf('PM worst-case (analítico) = %.1f°\n',     wcPM.PhaseMargin(1));
% 
% 
% %% Lag phase
% figure; hold on;
% bode(C*Gplanta);
% margin(C*Gplanta);
% title('Bode Lazo abierto – Controlador PID + Lag (planta nominal)');
% grid on;
% % mg = Inf
% % wg = Inf
% % mf = 76.8 deg
% % wf = 13.2 rad/s
% 
% wg_prima = 4.13; % rad/s
% GM_wg_prima = 5.5; % dB
% a = 10^(-GM_wg_prima / 20);
% T1 = 10 / (a*wg_prima);
% 
% % Lag phase
% Clag = (1+a*T1*s)/(1+T1*s);
% a, T1, Clag
% 
% %% Lead phase
% 
% % mf_req = 40;
% % mf_fact = 142;
% % 
% % theta_m = mf_req - mf_fact + 5;
% % b = (1+sind(theta_m))/(1-sind(theta_m));
% % freq_revision = -10*log10(b); % 24.27 dB
% % frecuencias = logspace(-1, 2, 200) 
% % 
% % figure; hold on;
% % bode(C * Gplanta, frecuencias);
% % 
% % wg_prima_adelanto = 1.66; % rad/s;
% % T2 = 1/(sqrt(b)*wg_prima_adelanto);
% % Clead = (1 + b*T2*s)/(1 + T2*s);
% % 
% % figure; hold on;
% % bode(C*Clead*Gplanta);