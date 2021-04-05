clear;

% Call function to connect to Coppeliasim : first must run program in
% coppeliasim that has a 'non-tread' child script' on some object (in
% example the object called "Dummy"
% 
% function sysCall_init()
%    -- do some initialization here
%    simRemoteApi.start(19999)
% end
%

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,50000,5);

% if connected to coppeliasim
if(clientID>-1)
    disp('Connected');
    
    % Initializw
    %    [returnCodem, left_Motor] =vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);

    % handle of the robot
    [returnCode, epuck] =vrep.simxGetObjectHandle(clientID, 'Dummy',vrep.simx_opmode_blocking);
    % handle of the ground
    [returnCode, ground] =vrep.simxGetObjectHandle(clientID, 'Ground',vrep.simx_opmode_blocking);
    
    % Other Codes;
    %    [returnCodev] = vrep.simxSetJointTargetVelocity(clientID,
    %    left_Motor, 0.1, vrep.simx_opmode_blocking);
    
    % obtain robot position and orientation relative to ground
    [returnCodep1,position1]=vrep.simxGetObjectPosition(clientID, epuck, ground, vrep.simx_opmode_streaming);
    [returnCodep1,orient]=vrep.simxGetObjectOrientation(clientID, epuck, ground, vrep.simx_opmode_streaming);
    %
    % get 2D laser data relative to the robot
    [returnCode, signalValue] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_streaming);

    % show data obtained
    figure(1); clf;
    subplot(211);
    h_plot = plot(0, 0, 'r'); axis equal; axis([-1 5 -1 5]);
    subplot(212);
    h1 = plot(0,0,'b.'); hold on;
    h2 = plot(0,0,'r.');
    h3 = plot(0,0,'k.');
    h4 = plot(0,0,'c.'); %hold off;
    axis equal; axis([-5 5 -5 5]);
    
%    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    X = []; z=[];
    for i=1:1000
        disp(i);
        % get 2D laser data relative to the robot
        [returnCode, data] = vrep.simxGetStringSignal(clientID,'measuredDataAtThisTime',vrep.simx_opmode_buffer);
        sensordata=vrep.simxUnpackFloats(data);
        % reshape data into 3 rows x,y,z of detect point cloud
        sensordata = reshape(sensordata,3,length(sensordata)/3);

        % get position and orientation
        [returnCodep2,position]=vrep.simxGetObjectPosition(clientID, epuck, ground, vrep.simx_opmode_buffer);
        [returnCodep1,orient]=vrep.simxGetObjectOrientation(clientID, epuck, ground, vrep.simx_opmode_buffer);        
        x = position(1); 
        y = position(2); 
        th = orient(3);
        
        xdata = []; ydata=[];
        xdata1=[]; ydata1=[]; 
        
        for j=1:size(sensordata,2)
            xray = sensordata(1,j); 
            yray = sensordata(2,j); 
            
            % get distance and angle data from laser
            ang_laser(j) = atan2(yray,xray);
            dis_laser(j) = sqrt((xray)^2 +(yray)^2);
            
            % for plot
            xdata = [xdata x x+dis_laser(j)*cos(th+ang_laser(j)) NaN];
            ydata = [ydata y y+dis_laser(j)*sin(th+ang_laser(j)) NaN];
        end
        
        h_plot.XData = xdata;
        h_plot.YData = ydata;
        
        % rotate laser data to ground frame 
        [xdata1, ydata1] = rotatez(sensordata,orient(3));             

        h1.XData = xdata1;
        h1.YData = ydata1;

%         drawnow;
        pause(0.2);
        
        % collect data
        X = [X [x;y;th] ];
        z(:,:,i) = [dis_laser; ang_laser];
    end
    %    [returnCodemotor] = vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0, vrep.simx_opmode_blocking);
    vrep.simxFinish(-1);
end

%vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.delete();

% save data to file
save 'data_vrep1.mat' X z 

function [x,y]=rotatez(data, a)
    for i=1:size(data,2)
        x(i)=data(1,i)*cos(a) - data(2,i)*sin(a);
        y(i)=data(1,i)*sin(a) + data(2,i)*cos(a);
    end
end
