s = tf("s");
Kp = 0.1;
Groot = 3.687e-6/(s^3 + 0.004853*s^2 + (1.104e-5 + 3.687e-6*Kp)*s);

a = 4.5 / 180;

figure
hold on
sgrid(0.69, 5)
tsline = -a + (-15:1:15)*1i;
plot(tsline, '--r');
rlocus(Groot)