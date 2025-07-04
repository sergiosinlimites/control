%% Datos iniciales
clc; clear;
m = 0.01165; % kg
m_adicional = 0.028 - m; % kg
l = 0.145; % m
g = 9.81; % m/s^2
I = (1/3*m + m_adicional)*l^2; % usando ejes paralelos + masa puntual
I = I * 85;
inercia = I;
punto_operacion = 80; % grados
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