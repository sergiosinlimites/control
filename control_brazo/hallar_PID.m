m = 0.01165; % kg
m_adicional = 0.028 - m; % kg
l = 0.145; % m
g = 9.81; % m/s^2
I = (1/3*m + m_adicional)*l^2; % usando ejes paralelos + masa puntual
I = I * 85;
punto_operacion = 40; % grados
k = (m*g*l/(2*I) + m_adicional * g * l / I);
alpha_experimental = 2.51e-2; % obtenido de los promedios del experimento de dejar caer el brazo
B = 2*I*alpha_experimental;


%% % Controlador PD I
syms s Kp Kd real

% Planta simbólica
G_sym = K_pwm_F * (l/I) / (s^2 + (B/I)*s + k);

% Controlador PD
C_sym = Kp + Kd * s;

% Lazo abierto simbólico
% Lazo_abierto = C_sym * G_sym;

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


% Definir variable de Laplace
s = tf('s');

% Fijar Kp
Kp = 1;

Q = 22574358785954750000*s;
P = 5629499534213120000*s^2  + 282600876617498624*s + 22574358785954750000*Kp + 4910752643630541875;

GCL = Q / P;



% Root locus
figure; hold on;
ts = 1;
tsline = -4.5/ts + (-15:1:15)*1i;
plot(tsline, '--r');
rlocus(GCL);
title('Root Locus para Kd (Kp elegido)');
xlabel('Parte Real'); ylabel('Parte Imaginaria');
grid on;
