% snippet to update goemetry for ke calculation

counter = counter+1;


% geometry input
%% Interpolate servo angles and body orientation
quat_time = robot_quat_list(i,1);

pitch_ang_L = interp1(robot_angle_rect(:,1),robot_angle_rect(:,2),quat_time, 'linear',robot_angle_rect(1,2));
pitch_ang_R = interp1(robot_angle_rect(:,1),robot_angle_rect(:,3),quat_time, 'linear',robot_angle_rect(1,3));

roll_ang_L = interp1(robot_angle_rect(:,1),robot_angle_rect(:,4),quat_time, 'linear',robot_angle_rect(1,3));
roll_ang_R = interp1(robot_angle_rect(:,1),robot_angle_rect(:,5),quat_time, 'linear',robot_angle_rect(1,5));

vib_ang = interp1(vib_angle_rect(:,1),vib_angle_rect(:,2),quat_time);

% Body rotation measured by the IMU
R_body = q2r_righting([robot_quat_list(i,5),robot_quat_list(i,2:4)]);




%% M345/Head

HeadOrigin_HeadCom = [77.46; -1.205 ; 7.186];

% M5
robot(M5).position = [0 0 0]';
robot(M5).orientation = eye(3); 

% HEAD
robot(HEAD).orientation= eye(3);
robot(HEAD).position = robot(M5).position  + rel_config(HEAD,4:6)';

% M3-M4
robot(M3).orientation = eye(3); 
robot(M4).orientation = eye(3);
robot(M3).position = robot(M5).position  + rel_config(M3,4:6)';
robot(M4).position = robot(M5).position  + rel_config(M4,4:6)';   

%% M1-M2 
robot(M1).orientation = robot(M3).orientation*EulerZYX_Fast([ 0 -(pitch_ang_R) 0]*pi/180); 
robot(M2).orientation = robot(M4).orientation*EulerZYX_Fast([ 0 -(pitch_ang_L) 0]*pi/180);

M3Origin_M1RotCenter = [ 5.91;  0; 1.63];    M4Origin_M2RotCenter = [ 5.91;  0; 1.63];   
M1RotCenter_M1Origin = [37.1 ; -2; 6.51];    M2RotCenter_M2Origin = [37.1 ;  2; 6.51];

robot(M1).position =   robot(M3).position + ...
                      robot(M3).orientation*M3Origin_M1RotCenter + ...
                      robot(M1).orientation*M1RotCenter_M1Origin;                       
robot(M2).position =   robot(M4).position + ...
                      robot(M4).orientation*M4Origin_M2RotCenter + ...
                      robot(M2).orientation*M2RotCenter_M2Origin;


%% WINGS
robot(W1).orientation = robot(M1).orientation*EulerZYX_Fast([roll_ang_R 0 0]*pi/180);
robot(W2).orientation = robot(M2).orientation*EulerZYX_Fast([-roll_ang_L 0 0]*pi/180);

M1Origin_W1RotCenter = [0;9.21;0];                M2Origin_W2RotCenter = [0;-9.21;0];
W1RotCenter_W1Origin = [-84.63; -14.31; 17.79];     W2RotCenter_W2Origin = [-84.63;  14.31; 17.79];
W1RotCenter_W1Com    = [-79.42; -39.8 ; -7.06];     W2RotCenter_W2Com    = [-79.42;  39.8 ; -7.06];  

robot(W1).position =   robot(M1).position + ...
                      robot(M1).orientation*M1Origin_W1RotCenter+ ...
                      robot(W1).orientation*W1RotCenter_W1Origin;                       
robot(W2).position =   robot(M2).position + ...
                      robot(M2).orientation*M2Origin_W2RotCenter+ ...
                      robot(W2).orientation*W2RotCenter_W2Origin;


%% TAIL-PENDULUM       
robot(TAIL).orientation = robot(M5).orientation*EulerZYX_Fast([0 0 vib_ang]*pi/180);
robot(PEN).orientation = robot(M5).orientation*EulerZYX_Fast([0 0 vib_ang]*pi/180);

M5Origin_TailRotCenter = [0; 0; 15];
TailRotCenter_TailOrigin = [-35;0;0];
robot(TAIL).position =   robot(M5).position + ...
                        robot(M5).orientation*M5Origin_TailRotCenter +...   
                        robot(TAIL).orientation*TailRotCenter_TailOrigin;

M5Origin_PenRotCenter = [0; 0; 30];
TailRotCenter_TailOrigin = [-64.5;0;0];
robot(PEN).position =   robot(M5).position + ...
                        robot(M5).orientation*M5Origin_TailRotCenter +...   
                        robot(TAIL).orientation*TailRotCenter_TailOrigin;
                    



%% apply body rotation and ground contact constraint / update pose for part frames                     
z_points = [];
all_points = [];

% stack all points
for ob =  1:length(entities)       

  % Concatenate all the points in the body      
  all_points = [all_points, [robot(ob).orientation*EulerZYX_Fast(rel_config(ob,1:3)*pi/180)*surfObj{ob}.v +  robot(ob).position ]];
  % Rotate all the entities to make the current configuration
  robot(ob).Patch.Vertices = (R_body*[robot(ob).orientation*EulerZYX_Fast(rel_config(ob,1:3)*pi/180)*surfObj{ob}.v +  robot(ob).position ])';           
  % Accumulate all the Z coordinates
  z_points = [z_points ; robot(ob).Patch.Vertices(:,3)];       
end   


% Find the lowest lying points to shift up all the bodies by
minZ = min(z_points);


for ob = 1:length(entities)             

  % Shift parts to enfronce ground contact constraint
  robot(ob).Patch.Vertices(:,3) = robot(ob).Patch.Vertices(:,3) - minZ;
  % update rotation
  if(any(ob==[M1 M2])) %% rotation correct for M1-M2 to align with principal axes
       robot(ob).R(:,:,counter) = R_body*[robot(ob).orientation*EulerZYX_Fast([0 -16 0]*pi/180)];
  else       
       robot(ob).R(:,:,counter) = R_body*[robot(ob).orientation];
  end
  robot(ob).eul(counter,:) = EULERZYXINV_Grass(robot(ob).R(:,:,counter));       

  % update position of center of mass
  switch ob
      case HEAD   %% position correction for HEAD to align with center of mass
       tmpPos = R_body*(robot(M5).position + ...
                        robot(M5).orientation*HeadOrigin_HeadCom);
      case W1     %% position correction for W1 to align with center of mass
       tmpPos = R_body*(robot(M1).position + ...
                        robot(M1).orientation*M1Origin_W1RotCenter+ ...
                        robot(W1).orientation*(W1RotCenter_W1Com));
      case W2     %% position correction for W2 to align with center of mass
       tmpPos = R_body*(robot(M2).position + ...
                        robot(M2).orientation*M2Origin_W2RotCenter+ ...
                        robot(W2).orientation*(W2RotCenter_W2Com));
      otherwise
       tmpPos = R_body*robot(ob).position;
  end
  robot(ob).comPos(counter,:) = [tmpPos(1) , tmpPos(2), tmpPos(3)-minZ];
  robot(ob).mh.XData = tmpPos(1);
  robot(ob).mh.YData = tmpPos(2);
  robot(ob).mh.ZData = tmpPos(3)-minZ;

end


% Calculate the center of mass
center_of_mass = [];
all_points = [];
for ob = [M1 M2 M3 M4 M5 TAIL PEN W1 W2 HEAD]
  center_of_mass = [center_of_mass , mass(ob)*robot(ob).comPos(counter,:)'];
  all_points =  [all_points, robot(ob).Patch.Vertices'];       
end
center_of_mass = sum(center_of_mass,2) / sum(mass([M1 M2 M3 M4 M5 TAIL PEN W1 W2 HEAD]));
robot(11).mh.XData = center_of_mass(1);
robot(11).mh.YData = center_of_mass(2);
robot(11).mh.ZData = center_of_mass(3);
robot(11).mh.MarkerSize = 8;
robot(11).mh.MarkerFaceColor = 'r';

% keep pointcloud data
pcl.head.x(counter,:) = robot(HEAD).Patch.Vertices(:,1)';
pcl.head.y(counter,:) = robot(HEAD).Patch.Vertices(:,2)';
pcl.head.z(counter,:) = robot(HEAD).Patch.Vertices(:,3)';
pcl.w1.x(counter,:) = robot(W1).Patch.Vertices(:,1)';
pcl.w1.y(counter,:) = robot(W1).Patch.Vertices(:,2)';
pcl.w1.z(counter,:) = robot(W1).Patch.Vertices(:,3)';
pcl.w2.x(counter,:) = robot(W2).Patch.Vertices(:,1)';
pcl.w2.y(counter,:) = robot(W2).Patch.Vertices(:,2)';
pcl.w2.z(counter,:) = robot(W2).Patch.Vertices(:,3)';
