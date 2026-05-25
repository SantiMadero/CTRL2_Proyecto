% Definir la planta
z = tf('z', 0.09);
Gz = z^-1 * (2.665*z^-1 + 1.203*z^-2 + 1.12*z^-3) / ...
    (1 - 1.626*z^-1 + 0.6629*z^-2 + 0.005682*z^-3);

% Orden del sistema
n = 4;

% Función de transferencia deseada (deadbeat)
Td = z^(-n);

% Calcular controlador
Cz = Td / (Gz * (1 - Td));

% Simplificar
Cz = minreal(Cz);
disp('Controlador Deadbeat:')
Cz

% Verificar lazo cerrado
T_cl = feedback(Cz*Gz, 1);
step(T_cl)
title("C");