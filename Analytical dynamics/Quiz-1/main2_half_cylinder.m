%% Simulation of Half cylinder using ODE from Lagrange Equation %%
clear; clear; clc;

%% Set parameter to symbolic %%
syms q1 q2 q1_d q2_d q1_dd q2_dd th1 th2 th1_d th2_d th1_dd th2_dd R m g e

%% Set global system parameters %%
global R e g 
g = 9.81; R=0.3; e=0.20;

%% Calculating kinetic and potential energy %%
T_1 = (0.5*m*q1_d^2)*(R^2-2*e*R*cos(q1)+e^2);
V_1 = m*g*(R-e*cos(q1));
T_2 = 0.5*m*((R*q1_d-R*q2*q1_d*sin(q1)+R*(q1_d+q2_d)*cos(q1)-e*(q1_d+q2_d)*cos(q1+q2))^2+(-R*q2*q1_d*cos(q1)-R*(q1_d+q2_d)*sin(q1)+e*(q1_d+q2_d)*sin(q1+q2))^2);
V_2 = m*g*(R-R*q2*sin(q1)+R*cos(q1)-e*cos(q1+q2));


%% Solve the lagrange's equation using "Lagrange_721" funcion  %%
[X1,X2] = Lagrange_721(T_1,V_1,T_2,V_2);


%% Set parameter of system %%
% convert degree to radial and radial to degree
d2r = pi/180;
r2d = 180/pi;

% time
dt = 0.001; %0.001
tfinal = 10;
t=[0:dt:tfinal];

% initial conditions angle between -90,90 
x0=[0*d2r;0;-30*d2r;0];

% Call ODE integration
[t,x] = ode45(@half_state,t,x0);


% set plot 1 handle
figure(1);
subplot(211); cla;
h_plot1 = plot(0,0,0,0);
axis([0 tfinal -90 90]);
    
% set animation object handle
figure(1);
subplot(212); cla; hold on;
% draw black floor line
plot([-3 3], [0 0], 'k');

% create line in x'-y' coordinate
line1xp = [-R R];
line1yp = 0*line1xp;

% cirle in local frame x'-y';
phi = [-pi:pi/50:0];
circle_xp = R*cos(phi);
circle_yp = R*sin(phi);
%plot line
h_line = plot(0,0,0,0);
axis equal;
axis([-1 1 0 1]); 
set(gca,'visible','off')

%% Go through each time step %%

for i=1:10:length(t)
    th1 = x(i,1);
    th2 = x(i,3);
    % update plot 1
    % figure(1);
    subplot(211); cla;
    h_polt1 = plot(t(1:i),x(1:i,1)*r2d,t(1:i),x(1:i,3)*r2d)
    % h_plot1.XData = t(1:i);
    % h_plot1.YData = x(1:i,1)*r2d;
   

    
    % update plot 2 cycle 1
    for j=1:length(line1xp)
        r_linep = [line1xp(j); line1yp(j); 0; 1]; 
        r_line(:,j) = transform_A(th1,R)*r_linep;
        r_line1(:,j) = transform_A(th1,R)*r_linep;
        r_line2(:,j) = transform_B(th1,th2,R)*r_linep;
    end
    
    
    %set(h_line,'XData',r_line1(1,:), 'YData', r_line1(2,:));
    
    for j=1:length(circle_xp)
        r_circlep = [circle_xp(j); circle_yp(j); 0; 1]; 
        r_circle(:,j) = transform_A(th1,R)*r_circlep; 
        r_circle1(:,j) = transform_A(th1,R)*r_circlep; 
        r_circle2(:,j) = transform_B(th1,th2,R)*r_circlep;
    end
    
    % update both line and half circle
    %figure(1);
    subplot(212); hold on; cla;
    axis equal;
    axis([-1 1 0 1]); 
    set(gca,'visible','off')
    x1=[r_line(1,1) r_line(1,end) NaN r_circle(1,:)];
    y1=[r_line(2,1) r_line(2,end) NaN r_circle(2,:)];
    x2=[r_line2(1,1) r_line2(1,end) NaN r_circle2(1,:)];
    y2=[r_line2(2,1) r_line2(2,end) NaN r_circle2(2,:)];
    h_line=plot(x1,y1,x2,y2);
    axis equal;
    axis([-1 1 0 1]); 
    set(gca,'visible','off')
    %h_line.XData = [r_line(1,1) r_line(1,end) NaN r_circle(1,:)];
    %h_line.YData = [r_line(2,1) r_line(2,end) NaN r_circle(2,:)];

    drawnow;
    pause(dt);
end
hold off;




%% Transfer function 

function T12 = transform_A(th1, R)
    s1 = sin(th1); c1=cos(th1);
    T12 = [c1   s1 0   R*th1;
           -s1   c1  0   R;
           0    0   1   0;
           0    0   0   1];
end

function T12 = transform_B(th1,th2, R)
    s1 = sin(th1); c1=cos(th1);
    s2 = sin(th2); c2=cos(th2);
    T10 = [c1   s1 0   R*th1;
           -s1   c1  0   R;
           0    0   1   0;
           0    0   0   1];
    T11 = [c2   s2 0   R*th2;
          -s2   c2  0   R;
           0    0   1   0;
           0    0   0   1];
    T12 =T10*T11;
end

function y = half_state(t,x)


global R e g  x1 x2 x3 x4 
    
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);
    
    y(1) = x(2);
    y(2) = (e*g*sin(x1) - (R^2*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - R*g*sin(x1) - (e^2*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - R^2*x2^2*sin(x1) + e*g*sin(x1 + x3) - R^2*x2^2*x3*cos(x1) - R*g*x3*cos(x1) + 2*R^2*x2*x3*x4 + R*e*x2^2*sin(x1 + x3) + R*e*x4^2*sin(x1 + x3) + R*e*x2^2*sin(x1) + R*e*x4^2*sin(x3) - (R^2*cos(x1)*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - 2*R^2*x2*x4*sin(x1) - R*e*x3*x4^2*cos(x3) + (R*e*cos(x1 + x3)*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) + 2*R*e*x2*x4*sin(x1 + x3) + (2*R*e*cos(x3)*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - 2*R*e*x2*x3*x4*cos(x3) + (R*e*x3*sin(x3)*(e*g*sin(x1 + x3) - R^2*x2^2*x3 - R*g*sin(x1) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)))/((((R^2*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) + (e^2*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) + (R^2*cos(x1)*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - (2*R*e*cos(x3)*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - (R*e*cos(x1 + x3)*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)) - (R*e*x3*sin(x3)*(R^2 + e^2 + R^2*cos(x1) - R*e*cos(x1 + x3) - 2*R*e*cos(x3) - R*e*x3*sin(x3)))/(R^2 + e^2 - 2*R*e*cos(x3)))/(2*R^2*x3*sin(x1) - 2*e^2 - R^2*x3^2 - 2*R^2*cos(x1) - 3*R^2 + 2*R*e*cos(x1 + x3) + 2*R*e*cos(x1) + 2*R*e*cos(x3) + 2*R*e*x3*sin(x3)) + 1)*(2*R^2*x3*sin(x1) - 2*e^2 - R^2*x3^2 - 2*R^2*cos(x1) - 3*R^2 + 2*R*e*cos(x1 + x3) + 2*R*e*cos(x1) + 2*R*e*cos(x3) + 2*R*e*x3*sin(x3)));
    y(3) = x(4);
    y2 = y(2);
    y(4) = -(R^2*y2 + e^2*y2 - R*g*sin(x1) - R^2*x2^2*x3 + R^2*y2*cos(x1) + e*g*sin(x1 + x3) - R*e*y2*cos(x1 + x3) - 2*R*e*y2*cos(x3) + R*e*x4^2*sin(x3) + R*e*x2^2*x3*cos(x3) - R*e*x3*y2*sin(x3))/(R^2 + e^2 - 2*R*e*cos(x3));
    y=y';
   
end