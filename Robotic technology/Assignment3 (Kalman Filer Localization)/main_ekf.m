clear; close; clc;

% set sensor range
sensor_range = 8;

% constants
d2r = pi/180;
r2d = 1/d2r;

% create landmarks randomly
xmin = -5; xmax = 5;
ymin = -5; ymax = 5;
num_landmark = 20;

for i=1:num_landmark
    landmark(1,i) = rand*(xmax-xmin)+xmin; 
    landmark(2,i) = rand*(ymax-ymin)+ymin; 
end

% create random real starting pose (X)
dum = 4;
x = dum*(rand-0.5);
y = dum*(rand-0.5);
th = pi*(rand-0.5);
X = [x;y;th];
%X = [5; 0; pi/2];

% create random believed starting pose (mu)
dum2 = 8;
x = dum2*(rand-0.5);
y = dum2*(rand-0.5);
th = pi*(rand-0.5);
mu = [x;y;th];
%mu = X+[0.1;0.1;0.1];

% initialize sigMa
sigM = 0.01*ones(3,3);

alp = 0.1*[1 1 1 1];
sig_r = 0.05; sig_phi = 1*d2r;
Qt = diag([sig_r^2, sig_phi^2]);

% Create figure display
figure(1); clf;
plot(landmark(1,:), landmark(2,:),'ks'); % black * shapes 
axis equal;
axis(2*[xmin xmax ymin ymax]);
hold on;

% create plot handles for ploting
h_real = plot(X(1),X(2),'bo');    % real robot: blue o
h_sim = plot(mu(1),mu(2),'ro');     % predicted robot: red o
h_lm = plot(0,0); % detected landmark rays;
h_head_real = quiver(X(1),X(2),0.5*cos(X(3)),0.5*sin(X(3)),'b'); % real robot heading line
h_head_sim = quiver(mu(1),mu(2),0.5*cos(mu(3)),0.5*sin(mu(3)),'r'); % predicted robot heading line
h_real.MarkerSize=10;
h_sim.MarkerSize=10;

ht1= text(0,9,'');
ht2= text(0,8,'');
% ht3= text(0,7,'');

disp('Press any key to start');
pause;
disp('RUNNING!!');

% create moving steps
for i=1:200 
    
    % set control actions:
    transl =0.1; 
    rot1 = 0.02; %*(rand-0.5);
    rot2 = 0.02; %*(rand-0.1); 
    
    % 2. extract th
    th = mu(3,1);
    % 3. Linearlized model w.r.t. states
    Gt = find_Gt(th,transl,rot1);
    % 4. Linearlized model w.r.t. control input
    Vt = find_Vt(th,transl,rot1);
    % 5. 
    Mt = find_Mt(alp,transl,rot1,rot2);
    % 6. update predicted pose
    mu = mu + [transl*cos(th+rot1); transl*sin(th+rot1); rot1+rot2];
    % 7 update covarience
    sigM = Gt*sigM*Gt'+Vt*Mt*Vt';
    
    % Obtain real pose X after the moving actions
    X = X + [transl*cos(X(3,1)+rot1); transl*sin(X(3,1)+rot1); rot1+rot2] + rand(3,1)*0.01;
    
    % 8. measurement covariance Qt is set above 
    
    % 9. real measurement
    [z,lm_idx] = find_landmark(X,landmark, sensor_range);
    
    % matrices to store data for ploting below
    x_detect=[];
    y_detect=[];
    
    % NOW go to each landmark that is detected with the sensor range        
    for i=1:length(lm_idx)
        if z(i) <= sensor_range            
            % Linearlized measurement model
            % 10. obtain index of landmark 
            k = lm_idx(i); 

            % 11. + 12. sensor model
            z_bar = find_z_bar(mu, landmark,k);
            
            % 13. Jacobian fir measurement
            Ht = find_Ht(mu(1),mu(2),landmark(1,k),landmark(2,k)); 

            % 14.+15.
            St = Ht*sigM*Ht'+Qt;
            Kt = sigM*Ht'*inv(St);

            % find error
            % err_z = (z(:,i)-z_bar);
            err_z = correct_angle_err(z(:,i), z_bar);
%             if (norm(err_z)) > 1;
%                 disp(['Error =', num2str(err_z(1)) ',' num2str(err_z(2))] );
%                 disp(['z=' num2str(z(1,i)), ',' num2str(z(2,i))]); 
%                 disp(['z_bar=' num2str(z_bar(1)) ',' num2str(z_bar(2))]); 
%                 pause;
%             end
            % 16. update believed states
            mu = mu + Kt*err_z;
            % 17. update covariance 
            sigM = (eye(3)-Kt*Ht)*sigM;
            
            % Collect points to draw sensor detection on landmarks
            x_detect = [x_detect X(1) landmark(1,k) NaN];
            y_detect = [y_detect X(2) landmark(2,k) NaN];
        end
    end
    %%% Ploting %%%%%%%%%%%%%
    h_real.XData = X(1);
    h_real.YData = X(2);
    h_sim.XData = mu(1);
    h_sim.YData = mu(2);
    h_lm.XData = x_detect;
    h_lm.YData = y_detect;
    h_head_real.XData = X(1); h_head_real.YData = X(2); 
    h_head_real.UData = 0.7*cos(X(3)); h_head_real.VData = 0.7*sin(X(3));
    h_head_sim.XData = mu(1); h_head_sim.YData = mu(2); 
    h_head_sim.UData = 0.7*cos(mu(3)); h_head_sim.VData = 0.7*sin(mu(3));
    
    ht1.String = sprintf('X,Y,TH real:( %2.2f, %2.2f, %4.1f)', X(1),X(2),X(3));
    ht2.String = sprintf('X,Y,TH pre :( %2.2f, %2.2f, %4.1f)', mu(1), mu(2),mu(3));
    drawnow;
    %%% finish plot %%%%%%%%%%%%%%
    pause(0.1);
end


%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%
%-------------------
function Gt = find_Gt(th, transl, rot1)
    Gt = [1 0 -transl*sin(th+rot1); 
        0 1 transl*cos(th+rot1); 
        0 0 1];    
end

%-------------------
function Ht = find_Ht(x, y, land_x, land_y)
dis2 = (land_x-x)^2+(land_y-y)^2;
r = sqrt(dis2);
Ht = [-(land_x-x)/r -(land_y-y)/r 0;
    (land_y-y)/dis2 -(land_x-x)/dis2 -1];
end

%-------------------
function [z,lm_idx] = find_landmark(X,landmark,sensor_range)
    x = X(1); y= X(2); th= X(3);
    z=[];
    j=0;
    lm_idx=[];
    
    for i=1:length(landmark)
        dis2 = (x-landmark(1,i))^2+(y-landmark(2,i))^2;
        % check if the landmakr is within sensor range
        % if within -- > record it, if not --> do nothing
        if dis2 <= sensor_range^2
            j=j+1;
            z(1,j) = sqrt(dis2) + randn*0.01; % add randomness to sensor
            z(2,j) = atan2(y-landmark(2,i), x-landmark(1,i)) - th + randn*0.01; % add randomness to sensor
            lm_idx(j)=i;
        end
    end
    
end

%-------------------
function Mt = find_Mt(alp, transl, rot1, rot2)
    Mt = diag([alp(1)*rot1^2+alp(2)*transl^2, 
        alp(3)*transl^2+ alp(4)*(rot1^2+rot2^2),
        alp(1)*rot1^2+alp(2)*transl^2]);        
end

%-------------------
function Vt = find_Vt(th, transl, rot1)
    Vt = [-transl*sin(th+rot1) cos(th+rot1) 0;
        transl*cos(th+rot1) -sin(th+rot1) 0;
        1 0 1];    
end

%-------------------
function z_bar = find_z_bar(mu,landmark,i)
x = mu(1); y= mu(2); th= mu(3);

dis2 = (x-landmark(1,i))^2+(y-landmark(2,i))^2;
z_bar(1,1) = sqrt(dis2);
z_bar(2,1) = atan2(y-landmark(2,i), x-landmark(1,i)) - th;
end

function err = correct_angle_err(z,zbar)
dum = z-zbar;
if (dum > pi)
    err = dum - 2*pi;
elseif (dum < -pi)
    err = dum + 2*pi;
else
    err = dum; 
end
end
