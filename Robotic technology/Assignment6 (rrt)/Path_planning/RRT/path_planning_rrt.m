%% Robotic technology %%
% Assignment:6 paht planning using rrt 
% Mr.Wathanyu chaiya ID 630631081

%% RRT algorithm %% 
clear; close; clc;

% Set parameter
epsilon = 0.9;        %For select state
p_home = [30,25];     %Start position 
p_goal = p_home;      %Goal posiion 
in_flate = 0.05;

% Initialize map
xmax = 10;
ymax = 10;
resolution = 5;
grid_size = 1/resolution;
L_xmax = xmax / grid_size;
L_ymax = ymax / grid_size;
disp(['Grid size: ' num2str(grid_size)]);

map = binaryOccupancyMap(xmax,ymax,resolution);
create_obstacle(map)
inflate(map,in_flate)


map = occupancyMatrix(map);
map = uint8(~map);  

% Initialize plot
figure(1); clf; hold on             
rrt_plot = pcolor(map);
rrt_plot.EdgeColor='none';
colorMap = bone(10);
colormap(colorMap);

axis equal;
axis([1 L_xmax 1 L_ymax]);

% Initialize graph tree
node_index = 1;
source_node = [node_index];  
target_node = [];            
nodes_x(node_index) = p_home(1);
nodes_y(node_index) = p_home(2);

rrt_graph = graph(source_node,target_node);
rrt_plot = plot(rrt_graph, 'b*','XData', nodes_y, 'YData', nodes_x,'MarkerSize', 10, 'NodeLabel',{'start'});
rrt_plot2 = plot(rrt_graph, 'g^','XData', p_goal(2), 'YData', p_goal(1),'MarkerSize', 10, 'NodeLabel',{'goal'});
axis equal;
axis([1 L_xmax 1 L_ymax]);

disp('Where do you want to go ?')
pause

xin = 'x_goal: ';
yin = 'y_goal: ';
p_goal(2) = input(xin);
p_goal(1) = input(yin);

delete(rrt_plot2)
rrt_plot2 = plot(rrt_graph, 'g^','XData', p_goal(2), 'YData', p_goal(1),'MarkerSize', 10, 'NodeLabel',{'goal'});
axis equal;
axis([1 L_xmax 1 L_ymax]);

disp('Press any key to start')
pause
iterations = 1;

% Check stopping condition (goal reached)
while (any(p_home ~= p_goal))
    
    iterations = iterations + 1;
    disp(['Step: ' num2str(iterations)]);
    
    % select direction state
    x_rand = select_state(p_goal,epsilon,1,L_xmax,L_xmax);
    
    % select nearest neighbor to this current random state ('seuclidean', 'minkowski', or 'mahalanobis')
    for node = 1:node_index
        neighbors(node) = pdist([nodes_x(node),nodes_y(node);x_rand(1),x_rand(2)],'minkowski');
    end
    
    [dist, nearest_node] = min(neighbors);
    % state of the nearest neighbor
    x_near = [nodes_x(nearest_node), nodes_y(nearest_node)];

    % move towards x_rand position
    p_home = x_near + move(x_near,x_rand);

    % check if position is occupied
   
    if map(p_home(1), p_home(2)) ~= 0
        % check if the node already exists
        exists_node = false;
        for i=1:node_index
            if p_home(1) == nodes_x(node) && p_home(2) == nodes_y(node)
               exists_node = true;
               break
            end
        end

        if exists_node == false
            % add current state as a node to the graph tree
            node_index = node_index + 1;
            rrt_graph = addnode(rrt_graph,1);
            rrt_graph = addedge(rrt_graph,nearest_node,node_index);
            nodes_x(node_index) = p_home(1);
            nodes_y(node_index) = p_home(2);
        end
    end
    
    delete(rrt_plot)
    delete(rrt_plot2)
    rrt_plot = plot(rrt_graph, 'r','XData', nodes_y, 'YData', nodes_x,'NodeLabel',{}, 'LineWidth', 0.5000, 'MarkerSize', 4);
    grid on
    pbaspect([1 1 1])
    axis equal;
    axis([1 L_xmax 1 L_ymax]);
    pause(0.01)
end
hold off

% Use A* to retrieve the shortest path
spath = shortestpath(rrt_graph,1,node_index);
highlight(rrt_plot,spath,'NodeColor','g','EdgeColor','g', 'LineWidth', 5);
disp('Sucseed !!')

%% ====================== Function for systems ========================= %%

% Create occupied cells
function create_obstacle(map)

grid_size = 1/map.Resolution;

x = [0:2*grid_size:5]';
y = 3*ones(size(x));
setOccupancy(map, [x y], ones(size(x)))

x = [0:2*grid_size:3]';
y = 7*ones(size(x));
setOccupancy(map, [x y], ones(size(x)))

x = [5:2*grid_size:9]';
y = 5.5*ones(size(x));
setOccupancy(map, [x y], ones(size(x)))

y = [7:2*grid_size:10]';
x = 7*ones(size(y));
setOccupancy(map, [x y], ones(size(y)))

y = [5:2*grid_size:7]';
x = 3*ones(size(y));
setOccupancy(map, [x y], ones(size(y)))

y = [0:2*grid_size:4]';
x = 7.5*ones(size(y));
setOccupancy(map, [x y], ones(size(y)))

% inflate(map,0.05);

end

% From a uniform distribution
function x = select_state(x_goal,epsilon,dist,L_xmax,L_ymax)

    if rand<epsilon
        if dist == 1
            
            x = [randi([1,L_xmax]), randi([1,L_ymax])];
        elseif dist == 2
            x(1) = random('Normal',25,7.5);
            x(2) = random('Normal',25,7.5);
            for i=1:2
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        elseif dist == 3
            x(1) = random('Rayleigh',x_goal(1));
            x(2) = random('Rayleigh',x_goal(2));
            for i=1:2
               if x(i) < 1
                  x(i) = 1;
               elseif x(i) >50
                   x(i) = 50;
               end
            end
        end
    else
        x = x_goal;
    end
end


% Find angle of target
function angle = find_orientation(source,target)
    target(1) = target(1)-source(1);
    target(2) = target(2)-source(2);
    angle = atan2(target(1),target(2));
    if angle < 0
        angle = angle + 2*pi;
    end
end


% The move of data from random 
function delta = move(source,target)
    angle = find_orientation(source,target);
    delta(1) = sin(angle);
    delta(2) = cos(angle);
    for i = 1:2
        if 0 < delta(i) && delta(i) < 0.3535
            delta(i) = 0;
        elseif 0.3535 <= delta(i) && delta(i) < 1
            delta(i) = 1;
        elseif -0.3535 < delta(i) && delta(i) < 0
            delta(i) = 0;
        elseif -1 < delta(i) && delta(i) <= -0.3535
            delta(i) = -1;
        end
    end
end

%% =============================== End ================================= %%