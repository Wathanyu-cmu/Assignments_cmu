function [ X1,X2 ] = Lagrange_721( T1,V1,T2,V2 )
%LAGRANGE_ Summary of this function goes here
% By Wathanyu Chaiya
% Calculation Of Lagrange Equation
% Analytical Dynamics

%%
% Set parameter to symbolic
syms q1 q2 q1_d q2_d q1_dd q2_dd th1 th2 th1_d th2_d th1_dd th2_dd R m g e

%%

%Find L from equation 
L = (T1+T2-V1-V2); % From L = (T1+T2) - (V1+V2)

%%
% Calculating equation use partial diff
L_q1 = diff(L,q1);
L_q1 = subs(L_q1,{m, q1, q2, q1_d, q2_d, q1_dd, q2_dd},{1, th1, th2, th1_d, th2_d, th1_dd, th2_dd});

L_q2 = diff(L,q2);
L_q2 = subs(L_q2,{m, q1, q2, q1_d, q2_d, q1_dd, q2_dd},{1, th1, th2, th1_d, th2_d, th1_dd, th2_dd});

L_q1_d = diff(L,q1_d);
L_q1_d = subs(L_q1_d,{q1, q2, q1_d, q2_d, q1_dd, q2_dd},{th1, th2, th1_d, th2_d, th1_dd, th2_dd});

L_q2_d= diff(L,q2_d);
L_q2_d = subs(L_q2_d,{q1, q2, q1_d, q2_d, q1_dd, q2_dd},{th1, th2, th1_d, th2_d, th1_dd, th2_dd});

%%
% Calculating equation use total diff
syms q1(t) q2(t) q1_d(t) q2_d(t) q1_dd(t) q2_dd(t) R m g e d a y2 x1 x3 x2 x4

Lq1d_t = subs(L_q1_d ,{th1, th2, th1_d, th2_d, th1_dd, th2_dd},{q1(t), q2(t), q1_d(t), q2_d(t), q1_dd(t), q2_dd(t)});
Lq2d_t = subs(L_q2_d ,{th1, th2, th1_d, th2_d, th1_dd, th2_dd},{q1(t), q2(t), q1_d(t), q2_d(t), q1_dd(t), q2_dd(t)});

L_q1_dt = diff(Lq1d_t,t);
L_q1_dt = subs(L_q1_dt,{diff(q1(t), t),diff(q2(t), t), diff(q1_d(t), t),diff(q2_d(t), t)},{q1_d, q2_d, q1_dd, q2_dd});
Lq1_dtd = subs(L_q1_dt ,{m, q1(t), q2(t), q1_d(t), q2_d(t), q1_dd(t), q2_dd(t)},{1,th1, th2, th1_d, th2_d, th1_dd, th2_dd});

L_q2_dt = diff(Lq2d_t,t);
L_q2_dt = subs(L_q2_dt,{diff(q1(t), t),diff(q2(t), t), diff(q1_d(t), t),diff(q2_d(t), t)},{q1_d, q2_d, q1_dd, q2_dd});
Lq2_dtd = subs(L_q2_dt ,{m, q1(t), q2(t), q1_d(t), q2_d(t), q1_dd(t), q2_dd(t)},{1, th1, th2, th1_d, th2_d, th1_dd, th2_dd});

%%
%Lagrange Equation
Q_1 = (Lq1_dtd - L_q1 == 0);
Q_2 = (Lq2_dtd - L_q2 == 0);

X1 = solve(Q_1,th1_dd);
X1  = simplify(X1);
X2 = solve(Q_2,th2_dd);
X2  = simplify(X2);

X2 = subs(X2,{th1, th2, th1_d, th2_d, th1_dd} , {x1, x3, x2, x4 ,y2});
X1 = subs(X1,{m ,th1, th2, th1_d, th2_d, th2_dd} , {1, x1, x3, x2, x4, X2}) == y2; 
X1 = solve(X1,y2);

fid = fopen('Theta_1dd.txt', 'w');
fwrite(fid, char(X1), 'char');
fclose(fid);

fid1 = fopen('Theta_2dd.txt', 'w');
fwrite(fid1, char(X2), 'char');
fclose(fid1);

end

