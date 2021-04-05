close all; clear; clc; 

load sample_data.mat 

%% set parameter %%
x_os = 10;
y_os = 15;
size_li = 20;

%% create space for map %%
xmn = min(X(1,:),[],'all') -20; 
xmx = max(X(1,:),[],'all') + 20;
ymn = min(X(2,:),[],'all') -20; 
ymx = max(X(2,:),[],'all') + 20;

%% create grid %%
stepx = 1; stepy = 1;
x = [xmn:stepx:xmx]; 
y = [ymn:stepy:ymx];

%% initialize grid value 0.5 grey, 0 black, 1 white
C = 0.5*ones(length(y),length(x));

%% Plot original data points %%
[XGrid, YGrid ] = meshgrid(x,y);
figure(1); clf; hold on                            
h_grid = pcolor(XGrid,YGrid,C);
h_grid.EdgeColor='none';

colorMap = bone(20);
colormap(colorMap);

%% handles for plots %%
h_robot = plot(0,0,'bo','MarkerSize',10, 'LineWidth',2);  %robot body 
h_robothead = quiver(0,0,0,0,'b','LineWidth',2); % real robot heading line
%h_laser = plot(0,0,'g-.','LineWidth',1); % laser
h_laser = plot(0,0,'g.','MarkerSize',size_li); % laser

%% go through each data samples %%
for tt=1:size(z,3)
    xr = X(1,tt);
    yr = X(2,tt);
    thr = X(3,tt);
    
    % plot
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
        [x0,y0,x1,y1]= find_xy_s2e(xr,yr,thr,sen_dis,sen_ang);
        x_laser = [x_laser x0 x1 NaN];
        y_laser = [y_laser y0 y1 NaN];
        
        %% Bresenham function %%
        B_line = bresenham_line(x0,y0,x1,y1);
        B_line(isnan(B_line))=0;
        
        %% Upgrade Occupancy gridmap %%
        for e = 1:size(B_line,1)
            for i=B_line(e,1)
                for j=B_line(e,2) 
                    if sum(abs([i,j]-B_line(end,:))) == 0
                        if C((j+y_os)+1,(i+x_os)+1) == 0
                            C((j+y_os)+1,(i+x_os)+1)= 0.5;
                            C(j+y_os,i+x_os)= 0;
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
    
    h_laser.XData = x_laser;h_laser.YData = y_laser;
    h_grid.CData = C;
end

function [xs,ys,xe,ye] = find_xy_s2e(xr, yr, thr, sensor_dis, sensor_ang)
    xs = xr; 
    ys = yr;
    xe = xr + sensor_dis*cos(thr+sensor_ang);
    ye = yr + sensor_dis*sin(thr+sensor_ang);
end


