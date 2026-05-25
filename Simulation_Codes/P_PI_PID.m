gz = tf(amx3322);

Ts = gz.Ts;

%% =========================
%% CONTROLADOR P
%% =========================
Kp1 = 0.016421;

C_P = tf(Kp1,1,Ts);

figure;
step(feedback(C_P*gz,1));
title('Respuesta Controlador P');

%% =========================
%% CONTROLADOR PI
%% =========================
Kp2 = 0.010047;
Ti2 = 0.86113;

Ki2 = Kp2/Ti2;

% PI discreto
C_PI = tf([Kp2+Ki2*Ts  -Kp2],[1 -1],Ts);

figure;
step(feedback(C_PI*gz,1));
title('Respuesta Controlador PI');

%% =========================
%% CONTROLADOR PID
%% =========================
Kp3 = 0.022965;
Ti3 = 0.83718;
Td3 = 0.2093;

Ki3 = Kp3/Ti3;
Kd3 = Kp3*Td3;

% PID discreto
num = [Kp3+Ki3*Ts+Kd3/Ts  -(Kp3+2*Kd3/Ts)  Kd3/Ts];
den = [1 -1 0];

C_PID = tf(num,den,Ts);

figure;
step(feedback(C_PID*gz,1));
title('Respuesta Controlador PID');