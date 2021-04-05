clear;
figure(1); clf;
% creat occupancy grid map
xmax = 10; ymax = 10; resolution = 50; grid_size = 1/resolution;
map = binaryOccupancyMap(xmax,ymax,resolution);

create_obstacle(map);
figure(1); hold on;
show(map);

% starting point 
% Xs = 1; Ys = 1;
Xs = rand*xmax;  Ys = rand*ymax; 
h_start = plot(Xs, Ys, 'bo','MarkerSize',8,'MarkerFaceColor',[0 0 1]);

% goal point
%Xg = 9; Yg = 9;
Xg = rand*xmax;  Yg = rand*ymax; 

h_goal = plot(Xg, Yg, 'gp','MarkerSize',12,'MarkerFaceColor',[0 1 0]);
goal_err = grid_size*10;

% max distance
max_move = 20*grid_size;
pause;

nodes(1).x = Xs;
nodes(1).y = Ys;
%nodes(1).connected = [];

tt=1;
while tt<2000
    tt=tt+1;     
    disp(['step=' num2str(tt)]);
    % create uniform random node;
    xr = rand*xmax;
    yr = rand*ymax;
    
    % Find the closest node
    min_dis = 1000;
    min_node = 1;
    for i=1:size(nodes,2)
        dis = find_dis(xr,yr,nodes(i).x, nodes(i).y);
%        disp(['aaa i=' num2str(i) ' dis=' num2str(dis)]);
        if (dis < min_dis)
            min_dis = dis;
            min_node = i;
        end
 %       disp(['min node num = ' num2str(min_node)]);
    end
    [nodes(tt).x, nodes(tt).y] = get_new_node_coor(nodes(min_node).x,nodes(min_node).y, xr, yr, min_dis, max_move);
    nodes(tt).parent = min_node;
    
    % add to adjacent nodes
    %nodes(tt).connected = [nodes(tt).connected min_node];
    %nodes(min_node).connected = [nodes(min_node).connected tt];
    
    %    
    plot(nodes(tt).x,nodes(tt).y,'r*');
    pause(0.01);
    plot([nodes(min_node).x nodes(tt).x],[nodes(min_node).y nodes(tt).y],'r');
    pause(0.01);
    
    dis2goal = find_dis(nodes(tt).x,nodes(tt).y, Xg,Yg);
    if dis2goal < goal_err
        node_goal = tt;
        break;
    end
end  
xpath=[];
ypath=[];
node_dum = node_goal;
while (1)
    node_parent = nodes(node_dum).parent;
    xpath = [xpath nodes(node_dum).x nodes(node_parent).x NaN];
    ypath = [ypath nodes(node_dum).y nodes(node_parent).y NaN];
    
    node_dum = node_parent;
    if node_dum == 1 % starting node
        break;
    end
end
plot(xpath, ypath, 'b-','Linewidth',2);


%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%
function create_obstacle(map)

grid_size = 1/map.Resolution;

% Create occupied cells
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

inflate(map,0.05);

end

function dis = find_dis(x1,y1,x2,y2)
    dis=sqrt((x1-x2)^2+(y1-y2)^2);
end


function [x,y] = get_new_node_coor(x1,y1,x2,y2, min_dis, max_move)
    if min_dis < max_move
        x = x2;
        y = y2;
    else
        x = x1+(x2-x1)/min_dis*max_move;
        y = y1+(y2-y1)/min_dis*max_move;
    end
end
    
    