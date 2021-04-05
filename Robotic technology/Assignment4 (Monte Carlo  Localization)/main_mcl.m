%% ===================================================================== %%

% Monte Carlo Localization using particle filter
% Feature-based
% By Theeraphong Wongratapahisan
% Feb 2021

%% ===================================================================== %%

clear; close; clc

% set parameters
sensor_range = 5;       % sensor range
num_landmark = 20;      % number of landmark
num_part = 100;         % number of particles
num_part_min = 50;
percent_discount = 5;   %0-100%
sigSensor = 0.25;  %0.25
sigXY = 0.05;      %0.05
sigTH = 1*pi/180;  %1*pi/180;

% constants
d2r = pi/180;
r2d = 1/d2r;

% create landmarks randomly but assumed known
xmin = -5; xmax = 5;
ymin = -5; ymax = 5;

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

% Create sample particles
belX = [rand(2,num_part)*(xmax-xmin)+xmin; 2*pi*(rand(1,num_part)-0.5)];
wX = []; % weights

% Control input
tran =0.2;
rot1 = 2*pi/180;
rot2 = 2*pi/180;

% For display texts
XPre = 0;
YPre = 0;
THPre = 0;
idx_max=1;

% Create figure display
figure(1); clf;
plot(landmark(1,:), landmark(2,:),'ks'); % black * shapes
axis equal;
axis(2*[xmin xmax ymin ymax]);
hold on;
title('Simulation using monte carlo localization');

% create plot handles for ploting
h_real = plot(X(1),X(2),'bo');    % real robot: blue o
h_lm = plot(0,0,'r'); % detected landmark rays;
h_head_real = quiver(X(1),X(2),0.7*cos(X(3)),0.7*sin(X(3)),'b'); % real robot heading line
h_real.MarkerSize=10;
ht1= text(0,9,'');
ht2= text(0,8,'');
ht3= text(0,7,'');
h_particle = plot(belX(1,:), belX(2,:),'r.'); % plot only positions
drawnow;


disp('Press any key to start');
pause;
disp('RUNNING!!');


% create moving steps
for tt=1:200
    
    if tt <= 100
       num_part = num_part; 
    elseif tt > 100 && num_part - round(num_part*percent_discount/100) > num_part_min  
       num_part = num_part - round(num_part*percent_discount/100);
    else
       num_part = num_part_min;
    end
    % initialize empty weights
    Weight = zeros(1,num_part);
    % set control actions:
    
    % Move the robot
    X = new_pose(X,tran+sigXY*randn, rot1+sigTH*randn, rot2+sigTH*randn);
    
    % Move particles
    for j=1:num_part
        belX(:,j) = new_pose(belX(:,j),tran+sigXY*randn, rot1+sigTH*randn, rot2+sigTH*randn);
    end
    
    % Scan the sensor to detect landmark 
    [z,lm_idx] = find_landmark(X,landmark, sensor_range);
    
    % initialize vectors for displaying sensor beams 
    x_detect=[];
    y_detect=[];
    
    % check if any landmark is detected
    if ~isempty(z)
        % go to each particle and apply sensor model
        for j=1:num_part
            par_prop = 0;
            
            % go to each landmark
            for k = 1:length(lm_idx)
                kk = lm_idx(k); % landmark index;
                % find distance and angle
                z_bar = find_z_bar(belX(:,j), landmark, kk);
                % accumulate propablity for each landmark and each particle
                par_prop = par_prop + 1/(sigSensor*sqrt(2*pi))*exp(-0.5*(z_bar(1)-z(1,k))^2/sigSensor^2);
                par_prop = par_prop + 1/(sigSensor*sqrt(2*pi))*exp(-0.5*correct_angle_err(z(2,k),z_bar(2))^2/sigSensor^2);

                % Collect points to draw sensor detection on landmarks
                x_detect = [x_detect X(1) landmark(1,kk) NaN];
                y_detect = [y_detect X(2) landmark(2,kk) NaN];
            end
            % scale the weight base on landmarks found
            Weight(j) = par_prop/2/k;
        end

        % normalize the weight
        Weight = Weight/sum(Weight);
         %Weight =  Weight(Weight~=0);
        % obtain the maxinum weight
        [maxW,idx_max] = max(Weight);
        % for ploting texts
        XPre = belX(1,idx_max);
        YPre = belX(2,idx_max);
        THPre = belX(3,idx_max)*r2d; 

        % -----------  Resampling 
        % Get cummerative weight
        for ii=2:num_part
            Weight(ii) = Weight(ii)+Weight(ii-1);
        end
        
        % Resampling -- low variance sampling
        dJ = 1/num_part;
        rr = dJ*rand;
        
        psum = 0;
        belX_new = zeros(size(belX));
        ii = 1;
        for j = 1:num_part
            psum = rr + (j-1)*dJ;
            while (psum > Weight(ii))
                ii = ii+1;
            end
            belX_new(:,j) = belX(:,ii);
        end
        % new samplings' poses
        belX = belX_new;
        belX(:,num_part+1:end) = []; 
    else
        % if landmarks are not found by sensor...the best particle remain
        % the same
        XPre = belX(1,idx_max);
        YPre = belX(2,idx_max);
        THPre = belX(3,idx_max)*r2d; 
    end
    % update handles for plotting
    h_head_real.XData = X(1);
    h_head_real.YData = X(2);
    h_head_real.UData = 0.7*cos(X(3));
    h_head_real.VData = 0.7*sin(X(3));
    h_particle.XData = belX(1,:);
    h_particle.YData = belX(2,:);
    h_lm.XData = x_detect;
    h_lm.YData = y_detect;
    h_real.XData = X(1);
    h_real.YData = X(2);
    ht1.String = sprintf('Data of real:(X : %2.2f, Y : %2.2f, Th : %4.1f)', X(1),X(2),X(3)*r2d);
    ht2.String = sprintf('Data of pre: (X : %2.2f, Y : %2.2f, Th : %4.1f)', XPre, YPre, THPre);
    ht3.String = sprintf('(Num part: %d, Iteration: %d)', num_part,tt);
    drawnow;
    
    pause(0.1);
end
%}

%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%
function z_bar = find_z_bar(mu,landmark,i)
x = mu(1); y= mu(2); th= mu(3);

dis2 = (x-landmark(1,i))^2+(y-landmark(2,i))^2;
z_bar(1,1) = sqrt(dis2);
z_bar(2,1) = atan2(y-landmark(2,i), x-landmark(1,i)) - th;
end


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

function err = correct_angle_err(z,zbar)
dum = z-zbar;
if (dum > pi)
    err = 2*pi-dum;
elseif (dum < -pi)
    err = 2*pi+dum;
else
    err = dum; 
end
end

function Xnew = new_pose(X, tran, rot1, rot2)
    Xnew = X + [tran*cos(X(3)+rot1); tran*sin(X(3)+rot1); rot1+rot2];
end