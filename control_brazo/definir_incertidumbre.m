%% 1) Declara parámetros inciertos  (ej. ±10 %)
m  = ureal('m', 0.01165, 'Percentage', 10);
mad= ureal('mad',0.01735, 'Percentage', 10);
g  = ureal('g', 9.81   , 'Percentage', 1);   % gravedad muy precisa
L  = ureal('L', 0.145  , 'Percentage', 5);
I  = ureal('I', 0.0362 , 'Percentage', 15);
B  = ureal('B', 1.81e-4, 'Percentage', 20);

thStar = deg2rad(80);

%% 2) Planta incierta
s = tf('s');
Kgrav = ((m*g*L + 2*mad*g*L)/(2*I)) * cos(thStar);
Gunc  = L/I / (s^2 + (B/I)*s + Kgrav);

%% 3) Controlador (usa tus Kp, Ki, Kd)
Kp=10; Ki=3; Kd=3.19;
C     = pid(Kp,Ki,Kd);
Lunc  = C*Gunc;    % lazo abierto incierto

%% 4) Márgenes para *todas* las plantas:
marg = allmargin(Lunc);      % estructura con GM, PM, Wcg, Wcp
disp(marg)

%% 5) Peor–caso automáticamente
[worstGM, worstPM] = wcgain(Lunc);    % returns struct
fprintf('GM peor-caso = %.2f dB\n', 20*log10(worstGM.GainMargin(1)))
fprintf('PM peor-caso = %.1f°\n',     worstPM.PhaseMargin(1))
