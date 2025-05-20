syms s t
syms Ki a b real

Gplanta = 150/(s^3 + 184*s^2 + 760*s + 162);

C    = Ki/s * (a*s + 1)/(b*s + 1);
Gcl  = collect(simplify(C * Gplanta / (1 + C * Gplanta)), s);

G0   = subs(Gcl, s, 0);
e_p  = 1 - G0;

alpha0  = 150*Ki;
betha0  = 150*Ki;
alpha1  = 150*Ki*a + 162;
betha1  = 150*Ki*a;

e_v = limit((alpha0 - betha0)/alpha0 * t - (alpha0*betha1 - betha0*alpha1)/alpha0^2, t, Inf);
e_v = simplify(e_v);

ki_hallado_min = 27/(25*0.2);

%% Fijar valores
% Ki_def = 6;
% b = 0.01;
% s = tf("s");
% 
% ts = 10;
% tsline = -4.5/10 + (-15:1:15)*1i;
% plot(tsline, '--r');
% 
% sgrid(0.357, 500);
% 
% hold on;
% 
% Groot = (150*Ki_def*s)/(b*s^5+(184*b+1)*s^4+(760*b+184)*s^3+(162*b+780)*s^2+(162)*s+150*Ki_def);
% rlocus(Groot);

%% Respuesta al escalón del lazo cerrado
Ki = 6;
a = 4;
b = 0.01;
s = tf("s");
C_tf     = Ki_def/s * (a*s + 1)/(b*s + 1);
Gplant_tf= 150/(s^3 + 184*s^2 + 760*s + 162);
Gcl_tf   = feedback(C_tf * Gplant_tf, 1);

% figure;
% step(Gcl_tf);
% grid on;
% title('Respuesta al escalón del sistema en lazo cerrado');

%% Función rampa
t_ramp = 0:0.1:50;  

% Genera la señal rampa (unitaria)
r = t_ramp;        

% Simula la respuesta del sistema en lazo cerrado Gcl_tf
[y_ramp, t_out] = lsim(Gcl_tf, r, t_ramp);

%Dibuja la entrada y la salida
plot(t_out, y_ramp, 'b', t_out, r, 'r--','LineWidth',1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Amplitud');
title('Respuesta a entrada rampa');
legend('Salida y(t)','Entrada rampa r(t)','Location','Best');