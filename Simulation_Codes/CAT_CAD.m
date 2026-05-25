clc;
clear;
close all;
% Tiempo de muestreo
Ts = 0.09;

% Variable discreta z
z = tf('z', Ts);

% Planta discreta G(z)
G = (2.665*z^2 + 1.203*z + 1.12) / ...
    (z^4 - 1.626*z^3 + 0.6629*z^2 + 0.005682*z);

% Controlador en atraso
C_atraso = 0.0045*(z - 0.85)/(z - 0.965);

% Controlador en adelanto
C_adelanto = 0.0255*(z - 0.725)/(z - 0.04);

% Sistemas en lazo cerrado
T_atraso = feedback(C_atraso*G, 1);
T_adelanto = feedback(C_adelanto*G, 1);

% Grafica de respuesta al escalon
figure;
step(T_adelanto);
title('Respuesta al escalon en lazo cerrado controlador en adelanto');
xlabel('Tiempo [s]');
ylabel('Amplitud');

figure;
step(T_atraso);
title('Respuesta al escalon en lazo cerrado controlador en atraso');
xlabel('Tiempo [s]');
ylabel('Amplitud');