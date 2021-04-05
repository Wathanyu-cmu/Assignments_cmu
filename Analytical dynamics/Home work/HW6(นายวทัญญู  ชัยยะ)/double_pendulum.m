%% By Wathanyu chaiya %%
close all; clear; clc;

%% Set global system parameters %%
global l g
g = 9.81; l = 1;
 
%% Set maximum of degrees %%
deg = [90 ,-90];

%% Convert radians to degrees %%
d2r = pi/180;
r2d = 180/pi;

%% Simulation time %%
s_t = 0.01;
t_final = 5;
t = [0:s_t:t_final];

%% Initial conditions angle between -90,90 %% 
xs = [deg(1)*d2r;0;deg(2)*d2r;0];

%% Solving ODE of a double pendulum %%
[t,x] = ode45(@func_pendulum,t,xs);


%% Calculating joint coordinates for animation purposes%%
X = [l*sin(x(:,1)),   l*sin(x(:,1))+l*sin(x(:,3))];
Y = [-l*cos(x(:,1)), -l*cos(x(:,1))-l*cos(x(:,3))];

%% Convert radians to degrees %%
ang = x(:,[1,3])*r2d;

%% Set up first frame %%
figure('Color', 'white')
subplot(2,1,1)
plot(t, ang, 'LineWidth', 2)
hh1(1) = line(t(1), ang(1,1), 'Marker', '.', 'MarkerSize', 20, ...
    'Color', 'b');
hh1(2) = line(t(1), ang(1,2), 'Marker', '.', 'MarkerSize', 20, ...
    'Color', 'r');
xlabel('time (sec)')
ylabel('angle (deg)')

subplot(2,1,2)
hh2 = plot([0, X(1,1);X(1,1), X(1,2)], [0, Y(1,1);Y(1,1), Y(1,2)], ...
    '.-', 'MarkerSize', 20, 'LineWidth', 2);
axis equal
axis([-2*l 2*l -2*l 2*l])
ht = title(sprintf('Time: %0.2f sec', t(1)));

%% Get figure size %%
pos = get(gcf, 'Position');
width = pos(3);
height = pos(4);

%% Preallocate data (for storing frame data) %%
mov = zeros(height, width, 1, length(t), 'uint8');

%% Loop through by changing XData and YData %%
for id = 1:length(t)
    
    % Update graphics data. This is more efficient than recreating plots.
    set(hh1(1), 'XData', t(id), 'YData', ang(id,1))
    set(hh1(2), 'XData', t(id), 'YData', ang(id,2))
    set(hh2(1), 'XData', [0, X(id, 1)], 'YData', [0, Y(id, 1)])
    set(hh2(2), 'XData', X(id, :), 'YData', Y(id,:))
    set(ht, 'String', sprintf('Time: %0.2f sec', t(id)))
    
    % Get frame as an image
    f = getframe(gcf);
    
    % Create a colormap for the first frame. For the rest of the frames,
    % use the same colormap
    if id == 1
        [mov(:,:,1,id), map] = rgb2ind(f.cdata, 256, 'nodither');
    else
        mov(:,:,1,id) = rgb2ind(f.cdata, map, 'nodither');
    end
    drawnow;
    pause(s_t);
end


%% Create animated GIF %%
imwrite(mov, map, 'double_pendulum.gif', 'DelayTime', 0, 'LoopCount', inf)

 
