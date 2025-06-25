%% Datos iniciales
m = 0.01165; % kg
l = 0.145; % m
g = 9.81; % m/s^2
I = 1/12*m*l^3;

punto_operacion = 90; % grados

%% 
k = m*g*l*cosd(punto_operacion);
K_pwm_grados = 0.0382; % pendiente de la curva ángulo (radianes) vs pwm (%)

K_rad_grados = 180 / pi;

alpha_experimental = 2.38e-3; % obtenido de los promedios del experimento de dejar caer el brazo
B = 2*I*alpha_experimental; % hallado sabiendo que alpha = termino_de_s / 2

