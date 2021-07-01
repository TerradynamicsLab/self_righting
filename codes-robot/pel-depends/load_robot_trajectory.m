function [quatVer,servoPulse,vibPulse,eulerSmoothed1, eulerSmoothed2] = load_robot_trajectory(matName)
% LOADACTUALTRAJECTORY returns the list of body/wing angles/axes 
% that have been recorded from the robot data 

% choose a default file
if(nargin==0)
   matName = 'run5_90_90_90_90_15_4.bag.mat' 
end

% Load the matfile
matData = load(['../data-robot/raw_data/' matName]);
motorPulse = load('../data-robot/raw_data/motor_pulse_v2.mat');

% Container variables
quatVer = [];
gravVer  =[];
servoVer = matData.servo;

% Verify the quaternion data
for i = 1:size(matData.quat,1)
    if abs( norm(matData.quat(i,2:5)) - 1) > 0.01 
        continue;%% should i make this a nan ?
    else
        quatVer = [quatVer ; matData.quat(i,:)];
    end
end

% Verify the gravity data
for i = 1:size(matData.grav,1)
    if abs( norm(matData.grav(i,2:4)) - 9.81) > 0.4 
        continue;%% should i make this a nan ?
    else
        gravVer = [gravVer; matData.grav(i,:)];
    end
end

% generate the motor angle v/s time values
[servoPulse,  vibPulse] = generate_motor_pulse_v2(servoVer);



eulerList = zeros(size(quatVer,1),4);
eulerListDual = zeros(7,size(quatVer,1));
for i =  1:size(eulerList,1)
%    [ex ey ez] =  InverseEulerZYX(quat2rotm([quatVer(i,5),quatVer(i,2:4)] ));
   rpy = EULERZYXINV_Grass(q2r_righting(quatVer(i,2:5)));
   ex  = rpy(1); ey = rpy(2); ez = rpy(3);
   eulerList(i,:) = [quatVer(i,1), ex,ey,ez] ;
   
   % For fixing the euler angles -- CHECK WHAT THIS ACTUALLY DOES!
   rpy =  EULERZYXINV_DualAngles(q2r_righting([quatVer(i,5),quatVer(i,2:4)] ));
   eulerListDual(:,i) = [quatVer(i,1)  rpy]' ;   
   
end

[eulerSmoothed1 eulerSmoothed2] = AngleSmooth_V1(eulerListDual(2:7,:));

eulerSmoothed1 = [eulerListDual(1,:); eulerSmoothed1];
eulerSmoothed2 = [eulerListDual(1,:); eulerSmoothed2];


% % Graph the trajectory
% trajFigure = figure(5);
% subplot(3,1,1)
% hold on
% plot(quatVer(:,1), quatVer(:,2) , 'c', 'LineWidth', 2) ;
% plot(quatVer(:,1), quatVer(:,3) , 'm', 'LineWidth', 2) ;
% plot(quatVer(:,1), quatVer(:,4) , 'y', 'LineWidth', 2) ;
% plot(quatVer(:,1), quatVer(:,5) , 'k', 'LineWidth', 2) ;
% ylim([-1.1,1.1]);
% 
% hold off
% subplot(3,1,2)
% hold on
% plot(gravVer(:,1), gravVer(:,2) , 'r', 'LineWidth', 2) ;
% plot(gravVer(:,1), gravVer(:,3) , 'g', 'LineWidth', 2) ;
% plot(gravVer(:,1), gravVer(:,4) , 'b', 'LineWidth', 2) ;
% ylim([-1.1,1.1]*9.81);
% hold off
% 
% 
% 
% hold off
% subplot(3,1,3)
hold on
% % plot(servoVer(:,1), servoVer(:,2) , 'c', 'LineWidth', 2) ;
% % plot(servoVer(:,1), servoVer(:,3) , 'm', 'LineWidth', 2) ;
% % plot(servoVer(:,1), servoVer(:,4) , 'y', 'LineWidth', 2) ;
% plot(servoPulse(:,1), servoPulse(:,2) , 'c', 'LineWidth', 2) ;
% plot(servoPulse(:,1), servoPulse(:,3) , 'm', 'LineWidth', 2) ;
% plot(servoPulse(:,1), servoPulse(:,4) , 'y', 'LineWidth', 2) ;
% plot(servoPulse(:,1), servoPulse(:,5) , 'k', 'LineWidth', 2) ;
% xlim([0, max(gravVer(:,1))]);
% % ylim([-1.1,1.1]*9.81);
% hold off

rectifiedEuler = eulerList(:,1:4);
angleDifference= adiff_righting(eulerList(:,2:4)*pi/180);
for i = 2:size(angleDifference,1)
        rectifiedEuler(i+1,2:4) = (wrapToPi(angleDifference(i,:))*180/pi+rectifiedEuler(i,2:4));
end

startingIndex = 1;
hold on 

%  plot(eulerList(startingIndex:end,1),eulerList(startingIndex:end,2) ,'r','LineWidth' , 2);
%  plot(eulerList(startingIndex:end,1),eulerList(startingIndex:end,3) ,'g','LineWidth' , 2);
%  plot(eulerList(startingIndex:end,1),eulerList(startingIndex:end,4) ,'b','LineWidth' , 2);
% 
% 
% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,2) ,'-c','LineWidth' , 1);
% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,3) ,'-m','LineWidth' , 1);
% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,4) ,'-y','LineWidth' , 1);
%  
%  
%  plot(eulerSmoothed2(1,startingIndex:end),eulerSmoothed2(4,startingIndex:end)*180/pi ,'*r','LineWidth' , 1);
%  plot(eulerSmoothed2(1,startingIndex:end),eulerSmoothed2(3,startingIndex:end)*180/pi ,'*g','LineWidth' , 1);
%  plot(eulerSmoothed2(1,startingIndex:end),eulerSmoothed2(2,startingIndex:end)*180/pi ,'*b','LineWidth' , 1);

% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,2) ,'-c','LineWidth' , 1);
% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,3) ,'-m','LineWidth' , 1);
% %  plot(rectifiedEuler(startingIndex:end,1),rectifiedEuler(startingIndex:end,4) ,'-y','LineWidth' , 1); 
%  
 
 
% %  legend('yaw' , 'pitch' , 'roll' ,'yaw' , 'pitch' , 'roll', 'Location', 'southwest')
%  title('Euler Angles');
%  xlabel('Time (s)');
%  ylabel('Angle (deg)');
%  box on 
%  hold off

eulerList = [eulerList, rectifiedEuler];



end