% Asume que tf1 ya está en workspace como tu planta original (LTI)
[num, den] = tfdata(tf1, "v");  % extrae los vectores de coeficientes
Gplanta    = tf(num, den);      % planta en tf “num/den”

%% 3) Elección de Kp y Construcción del Lazo Cerrado
s = tf("s");
Kp = 0.1;                             % <- cámbialo por el valor que prefieras
Ki = 1;

C = Kp + Ki/s;
% feedback construye automáticamente:
%   Gcl(s) = Kp*Gplanta / (1 + Kp*Gplanta)
Gcl = C * Gplanta / (1 + C * Gplanta);

%% 2) Root‐locus del Lazo cerrado
%figure(1)

figure
hold on
sgrid(0.69, 5)
tsline = -0.9 + (-15:1:15)*1i;
plot(tsline, '--r');
rlocus(Gcl)


% líneas de damping=0.69 y wn=5
%tsline = -0.9 + (-15:1.5:15)*1i;    
%plot(tsline, "--r", "LineWidth",1)
%title("Root‐Locus de K_p · G_{cl}(s)")
%xlabel("\sigma")
%ylabel("j\omega")
%grid on
%hold off

%% 4) Polos del Lazo Cerrado
%p_cl = pole(Gcl);
%disp("Polos del sistema en lazo cerrado:");
%disp(p_cl)

%% 5) Respuesta al Escalón (opcional)
%figure(2)
%step(Gcl)
%title(sprintf("Respuesta al escalón para K_p = %.2f", Kp))
%grid on
