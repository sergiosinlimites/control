syms s t
syms Ki Kp real

[num, den] = tfdata(tf1, "v"); % FT de la planta obtenida con 82% de similitud
Max_diff = 4;
M_p = (Max_diff)/32; % Sobrepico considerado
zita = 0.741; % Amortiguamiento máximo para sobrepico
ts = 600; % Tiempo de asentamiento considerado (no hay un valor exacto según investigaciones)

Gplanta = 0.01927 / (s^2 + 5.9*s + 0.05495);

%% Con un controlador P
syms s Kp;
Gcl_sym_P = simplify((Gplanta * Kp) / (1 + Gplanta * Kp));

Kp = 1;
C_P = Kp;
% Lazo cerrado
s = tf("s");
Gcl_P = (1927*Kp)/(100000*s^2 + 590000*s + 1927*Kp + 5495);

%% Con un controlador PI
syms s
syms Ki Kp real
% Controlador
C_PI    = Ki/s + Kp;
% Lazo cerrado
Gcl_PI  = collect(simplify(C_PI * Gplanta / (1 + C_PI * Gplanta)), s);

%% Root Locus para lazo cerrado con controlador proporcional
% tsline = -4.5/ts + (-15:1:15)*1i;
% plot(tsline, '--r');
% sgrid(zita, 10);
% hold on;
% rlocus(Gcl_P);
% Da una ganancia de 448 para no incluir senoidales y máxima de 775 máxima con senoidales.
%% Pruebas paso con controlador P
Kp = 70;
s = tf("s");
Gcl_P_tf = (1927*Kp)/(100000*s^2 + 590000*s + 1927*Kp + 5495);
step(Gcl_P_tf);

%% Root Locus para denominador del lazo cerrado con controlador PI
Kp_def = 30;
s = tf("s");
Groot = 1927/(100000*s^3+590000*s^2+(1927*Kp_def+5495)*s);
ts = 600;
tsline = -4.5/ts + (-15:1:15)*1i;
plot(tsline, '--r');

sgrid(0.741, 10);
hold on;
rlocus(Groot);
hold off;

%% Pruebas paso con controlador PI
Ki = 0.0983;
Kp = 5;
s = tf("s");
C_PI_tf = Ki/s + Kp;
Gplant_PI_tf = 0.01927/(s^2 + 5.9*s + 0.05499);
Gcl_PI_tf   = C_PI_tf * Gplant_PI_tf / (1 + C_PI_tf * Gplant_PI_tf);
step(Gcl_PI_tf);

syms s
Gcl_PI_simplificado = simplify((0.1927*s^4 + 1.142*s^3 + 0.03766*s^2 + 0.0002522*s)/(s^6 + 11.8*s^5 + 35.11*s^4 + 1.79*s^3 + 0.04068*s^2 + 0.0002522*s));

