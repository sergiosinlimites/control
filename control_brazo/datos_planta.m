%% Datos iniciales
clc; clear;
m = 0.01165; % kg
m_adicional = 0.02; % kg
l = 0.145; % m
g = 9.81; % m/s^2
I = 1/12*m*l^3;
I = I * 1220;
punto_operacion = 80; % grados

%% 
k = m*g*l*cosd(punto_operacion) + 2*m_adicional*g*l*cosd(punto_operacion);
K_pwm_rad = 0.0382; % pendiente de la curva ángulo (radianes) vs pwm (%)

K_rad_grados = 180 / pi;

alpha_experimental = 2.51e-2; % obtenido de los promedios del experimento de dejar caer el brazo
B = 2*I*alpha_experimental; % hallado sabiendo que alpha = termino_de_s / 2


%% 1) Carga los datos experimentales desde Excel
% Suponiendo que tu archivo se llama "ExperimentoFriccionP2.xlsx" y la
% hoja con los datos se llama "DEF"
M = readmatrix('datos_caida_brazo_exp_1.xlsx','Sheet','Hoja1');
t_exp     = M(:,1);      % columna de tiempos (s)
theta_exp = M(:,2);      % columna de ángulos (grados)
angulo_inicial = theta_exp(1) * pi / 180; % angulo inicial de la prueba, en radianes 


M = readmatrix('diagrama_pwm_vs_angulo.xlsx','Sheet','Hoja2');
F_exp     = M(:,1);      % columna de tiempos (s)
theta_exp_fuerza = M(:,2);      % columna de ángulos (radianes)

pwm_data    = linspace(0, 1000, length(t_exp))';
pwm_entrada = timeseries(pwm_data, t_exp);

%% 2) Lanza la simulación de tu modelo Simulink
% Asegúrate de que en tu modelo tengas un bloque To Workspace que
% vierta la señal angular en 'out.simout' (timeseries).
simOut  = sim('planta','ReturnWorkspaceOutputs','on');  % sustituye 'planta' por el nombre de tu .slx si es otro

%% 3) Extrae la señal simulada de la variable 'out'
ts  = simOut.get('simout');% timeseries
t_sim    = ts.time;        % tiempo de simulación
theta_sim = ts.data;       % ángulo simulado (grados)

% ts_fuerza = simOut.get('datos_fuerza_angulo__fuerza');
% ts_angulo = simOut.get('datos_fuerza_angulo__angulo');
% fuerza    = ts_fuerza.data;
% angulo    = ts_angulo.data;

%% 4) Barrido de PWM y extracción del ángulo en estado estacionario
pwm_vals      = F_exp;                       % tus valores experimentales de PWM
theta_sim_f   = zeros(size(pwm_vals));       % prealoca

for i = 1:numel(pwm_vals)
    % Prepara la entrada escalar
    simIn = Simulink.SimulationInput('planta');
    simIn = simIn.setVariable('pwm_entrada', pwm_vals(i));
    
    % Corre la simulación
    simOut_i = sim(simIn,'ReturnWorkspaceOutputs','on');
    
    % Extrae la serie de ángulo para esta simulación
    ts_ang_i = simOut_i.get('datos_fuerza_angulo__angulo');
    
    % Toma el último valor (estado estable)
    theta_sim_f(i) = ts_ang_i.data(end);
end

%% 4) Grafica ambas curvas superpuestas
figure; hold on;
plot(t_exp,     theta_exp, 'ro',   'MarkerSize',6, 'DisplayName','Datos Exp');
plot(t_sim,     theta_sim, 'b-',   'LineWidth',1.5,'DisplayName','Simulink');
xlabel('Tiempo (s)');
ylabel('Ángulo \theta (°)');
title ('Comparación: Datos Excel vs. Salida Simulink');
legend('Location','best');
grid on;

%% 5) Grafica la comparación PWM vs. ángulo
figure; hold on;
h1 = plot(F_exp,           theta_exp_fuerza, 'ro', 'MarkerSize',6);
h2 = plot(pwm_vals,        theta_sim_f,       'b-', 'LineWidth',1.5);

xlabel('Voltage (PWM)');
ylabel('Ángulo \theta (°)');
title ('Comparación: Datos Excel vs. Modelo Simulink (barrido de PWM)');
legend([h1 h2], {'Datos Exp Fuerza','Simulink Fuerza'}, 'Location','best');
grid on;