gz = tf(amx3322);
gc = d2c(gz);
k = 1;


gz_lc = feedback(gz*k,1);
%step(gz_lc);


K_crit = 0.045311439;
P_crit = 1.17e-4;


P1 = 0.5*K_crit;

P2 = 0.45*K_crit;

P3 = 0.6*K_crit;

%I1 = inf

I2 = P_crit/1.2;

I3 = 0.5*P_crit;

D1 = 0;

D2 = 0;

D3 = 0.125*P_crit;

