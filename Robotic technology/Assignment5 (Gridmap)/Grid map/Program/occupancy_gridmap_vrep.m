close all; clear; clc; 

load data_vrep.mat

%% Set parameter %%
x_os = 450;   % offset of x
y_os = 460;   % offset of y
size_li = 1; % Size of lidar plot 
step_g =  1;  % Grid size 

%% Normalize data convert m to cm %%
X(1,:) = round(X(1,:)*100,5);
X(2,:) = round(X(2,:)*100,5);
z(1,:) = round(z(1,:)*100,5);

%% create space for map %%
xmn = min(X(1,:),[],'all') -500; 
xmx = max(X(1,:),[],'all') +500;
ymn = min(X(2,:),[],'all') -500; 
ymx = max(X(2,:),[],'all') +500;

%% create grid %%
x = [xmn:step_g:xmx];
y = [ymn:step_g:ymx];


%% Initialize grid value 0.5 grey, 0 black, 1 white %%
C = 0.5*ones(length(y),length(x));

%% Plot original data points %%
[XGrid, YGrid ] = meshgrid(x,y);
figure(1); clf; hold on             
h_grid = pcolor(XGrid,YGrid,C);
h_grid.EdgeColor='none';
colorMap = bone(20);
colormap(colorMap);

%% handles for plots %%
h_robot = plot(0,0,'bo','MarkerSize',10, 'LineWidth',1);  %robot body 
h_robothead = quiver(0,0,0,0,'b-','LineWidth',1); % real robot heading line
%h_laser = plot(0,0,'g-.','LineWidth',1); % laser
h_laser = plot(0,0,'g.','MarkerSize',size_li); % laser
axis equal;
axis([-50 500 -50 500]);

%% convert cm to m for plot %%
xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',xt/100)
yt = get(gca, 'YTick');
set(gca, 'YTick',yt, 'YTickLabel',yt/100)

for tt=1:size(z,3)
    
    xr = X(1,tt);
    yr = X(2,tt);
    thr = X(3,tt);
    
    %% Plot %% 
    h_robot.XData = xr; 
    h_robot.YData = yr; 
    h_robothead.XData = xr; h_robothead.YData = yr;
    h_robothead.UData = 5*cos(thr); 
    h_robothead.VData = 5*sin(thr);
    drawnow;
    
    x_laser = [];
    y_laser = [];

    for phi = 1:size(z,2)
     
        sen_dis = z(1,phi,tt); % distance
        sen_ang = z(2,phi,tt); % angle relative to robot
        
        %% for laser plotting %%
        [xs,ys,xe,ye]= find_xy_s2e(xr,yr,thr,sen_dis,sen_ang);
        
        xe = round(xe,5);
        ye = round(ye,5);
        x_laser = [x_laser xs xe NaN];
        y_laser = [y_laser ys ye NaN];
        
        %% Bresenham function %%
        B_line = bresenham_line(xs,ys,xe,ye);
        
        %% Upgrade Occupancy gridmap V1 %%
%         for e = 1:2:size(B_line,1)
%             for i = B_line(e,1)
%                 for j = B_line(e,2) 
%                     if sum(abs([i,j]-B_line(end,:))) == 0
%                        C(j+y_os,i+x_os) = 0;
%                        C((j+y_os)-1,(i+x_os)-1) = 0;
%                        C((j+y_os)+1,(i+x_os)+1) = 0;
%                     else   
%                        C(j+y_os,i+x_os) = 1;
%                     end
%                 end        
%             end
%         end
 %% Upgrade Occupancy gridmap V2 %%      
        for e = 1:size(B_line,1)
            
            for i=B_line(e,1)
                for j=B_line(e,2) 
                    if sum(abs([i,j]-B_line(end,:))) == 0
                        if C((j+y_os)+1,(i+x_os)+1) == 0
                            C(j+y_os,i+x_os)= 0.5;
                        else
                            C(j+y_os,i+x_os)= 0;
                        end   
                    else
                        if C(j+y_os,i+x_os)== 0
                            C(j+y_os,i+x_os)= 0;  
                        else
                            C(j+y_os,i+x_os) = 1;
                        end   
                    end
                end        
            end
        end
        
    end
   
    %% Plot %%
    h_laser.XData = x_laser;    h_laser.YData = y_laser;
    h_grid.CData = C;
    
end

function [xs,ys,xe,ye] = find_xy_s2e(xr, yr, thr, sensor_dis, sensor_ang)
    xs = xr; 
    ys = yr;
    xe = xr + sensor_dis*cos(thr+sensor_ang);
    ye = yr + sensor_dis*sin(thr+sensor_ang);
end


