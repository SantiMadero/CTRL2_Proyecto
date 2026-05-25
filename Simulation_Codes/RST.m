clear; clc; close all;

%% Tiempo de muestreo
Ts = 0.09;

%% Planta en q^-1
% G(q^-1) = q^-2 B(q^-1) / A(q^-1)

A = [1  -1.626   0.6629   0.005682];
B = [2.665   1.203   1.12];

d = 2;   % retardo equivalente q^-2

%% Controlador RST diseñado

R = [1  -0.786781   0.085530  -0.182727  -0.116022];

S = [0.125275  -0.190227   0.073457   0.000432];

% Ganancia para referencia
T0 = sum(S);
T = T0;

%% Polinomio característico de lazo cerrado
% A(q^-1)R(q^-1) + q^-d B(q^-1)S(q^-1)

AR = conv(A,R);
BS = conv(B,S);

% Agregar retardo q^-d al término B*S
BSd = [zeros(1,d) BS];

% Igualar tamaños antes de sumar
nmax = max(length(AR), length(BSd));
AR  = [AR  zeros(1,nmax-length(AR))];
BSd = [BSd zeros(1,nmax-length(BSd))];

Alc = AR + BSd;

%% Transferencia Y/R
% Y/R = q^-d B(q^-1)T(q^-1) / Alc(q^-1)

BT = conv(B,T);
num_y = [zeros(1,d) BT];
num_y = [num_y zeros(1,length(Alc)-length(num_y))];

%% Transferencia U/R
% U/R = A(q^-1)T(q^-1) / Alc(q^-1)

AT = conv(A,T);
num_u = AT;
num_u = [num_u zeros(1,length(Alc)-length(num_u))];

%% Simulación discreta con entrada escalón

t = 0:Ts:5;
r = ones(size(t));

% filter trabaja directamente con polinomios en q^-1
y = filter(num_y, Alc, r);
u = filter(num_u, Alc, r);

%% Gráfica discreta de la salida

figure;
stairs(t,y,'LineWidth',1.5);
grid on;
title('Respuesta al escalón del lazo cerrado RST');
xlabel('Tiempo [s]');
ylabel('Salida y(k)');
xlim([0 t(end)]);

%% Gráfica discreta de la señal de control

figure;
stairs(t,u,'LineWidth',1.5);
grid on;
title('Señal de control u(k)');
xlabel('Tiempo [s]');
ylabel('Control u(k)');
xlim([0 t(end)]);

%% También se crean las funciones de transferencia en q^-1

Gyr = tf(num_y, Alc, Ts, 'Variable', 'z^-1');
Gur = tf(num_u, Alc, Ts, 'Variable', 'z^-1');

disp('Funcion de transferencia Y/R en z^-1:');
Gyr

disp('Funcion de transferencia U/R en z^-1:');
Gur

%% Desempeño aproximado de la salida

info = stepinfo(y,t,1,'SettlingTimeThreshold',0.02);

fprintf('--- Desempeño del lazo cerrado ---\n');
fprintf('Sobreimpulso Mp = %.4f %%\n', info.Overshoot);
fprintf('Tiempo de establecimiento ts = %.4f s\n', info.SettlingTime);
fprintf('Valor final aproximado de y = %.4f\n', y(end));
fprintf('Valor final aproximado de u = %.4f\n', u(end));