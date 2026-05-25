load("C:\Users\Madero\Desktop\Uppies\amx3322.mat")
% --- Planta ---
Gz = tf(amx3322);
sys_ss = ss(Gz);
A = sys_ss.A; B = sys_ss.B; C = sys_ss.C; D = sys_ss.D;

% --- Verificar controlabilidad ---
Co = ctrb(A, B);
fprintf('Rango matriz de controlabilidad: %d\n', rank(Co));

% --- Diseño LQR ---
%Q = C'*C;
Q = eye(3);
R = 1;
[K, S, P] = dlqr(A, B, Q, R);
fprintf('Ganancias K: '); disp(K)
fprintf('Polos LC: '); disp(P)

% --- Prealimentación ---
N = 1 / (C * inv(eye(size(A)) - A + B*K) * B);

% --- Simulación ---
Acl = A - B*K;
Bcl = B*N;
sys_cl = ss(Acl, Bcl, C, D, 0.09);
figure; step(sys_cl); title('Respuesta al escalón en lazo cerrado controlador LQR')