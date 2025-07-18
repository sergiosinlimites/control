clc; clear; close all;

% Parámetros físicos
m   = 0.01165;    % masa del brazo [kg]
mad = 0.01735;    % masa adicional [kg]
g   = 9.81;       % gravedad [m/s^2]
L   = 0.145;      % longitud [m]
B   = 1.812642814729166e-04;  % coef. de fricción [N·m·s]
Theta_star = deg2rad(30);    % punto de operación (rad)

% Inercia total del sistema
I = 0.0362;

% Término gravitacional con cos(theta*)
Kgrav = ((m*g*L + 2*mad*g*L) / (2*I)) * cos(Theta_star);

% Variable de Laplace
s = tf('s');

% Función de transferencia corregida
G = (L/I) / (s^2 + (B/I)*s + Kgrav);

% Mostrar función de transferencia
disp('Función de transferencia corregida con cos(theta*):');
G
%% % PARTE SIMBÓLICA
syms s Kp Kd real

% Planta simbólica
G_sym = (L/I) / (s^2 + (B/I)*s + Kgrav);

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
%% 

% Definir variable de Laplace
s = tf('s');

% Fijar Kp
Kp = 10;

% Numerador (constante Q)
Q = 208967022709991014400 * s;

% Denominador p(s)
p = 52169698083459825664 * s^2  + 261229360159042513 * s + 208967022709991014400 * Kp + 41143111907590589568;
% Función de transferencia GCL(s)
GCL = Q / p;

% Root locus
% rlocus(GCL);
% title('Root Locus para Kd (Kp elegido)');
% xlabel('Parte Real'); ylabel('Parte Imaginaria');
% grid on;
% hold on;


% --- Después de haber hecho el raíz-locus ---

% Asegúrate de que 's' sea la TF, no símbolo:
clear s
s = tf('s');

% Parámetros del controlador
Kp = 10;
Kd = 3.19;
Ki = 0.1;

% Controlador PD y planta G ya definidos antes
C = (Kp + Kd*s + Ki/s);       % Controlador PID
L = C * G;           % Lazo abierto

% Lazo cerrado con realimentación unitaria
T_cl = feedback(L, 1);

% Respuesta al escalón
figure;
step(T_cl);
title('Respuesta al escalón del sistema en lazo cerrado');
xlabel('Tiempo (s)');
ylabel('Amplitud');
grid on;


