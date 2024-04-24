s = tf('s');
j=0.0019;
b=0.00826;

kt=0.6741;
ke=0.6741;
% motor 1:
R=12;

L=90*10^-6;

T=1/200;
td = 1;
Ti=7 ;

K_p = 1;

%controller
G=K_p*(1+td*s+Ti*1/s);
%delay
Gd= 1/(T/(2*s)+1);
%plant
sys = (k)/((((j*s+b)*(L*s+R)+k^2)))

rlocus(sys*Gd*G)

K_p = 3.8;
G=K_p*(1+td*s+Ti*1/s);
syscl = sys*Gd*G/(1+sys*Gd*G);

plot(sys)

