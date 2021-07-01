%% code to test if the robot geometry works

% close all

addpath(genpath('obj'))

% load the robot
load_robot_model

% generate_entities
generate_robot_entities

% update robot geometry
wing_angles=  [-2.0821   -2.0631];
vibration_angle = 0;

i = 1;counter = 0
br = -172*pi/180; bp = 32*pi/180 ; by = 0;
update_robot_geometry


%% plot all frames
view(144,24)
for i = 1:length(entities)
    
   col_ax = {'r','g','b'};
    %xaxis
   for j = 1:3
       robot(i).ax_h(j)= plot3([0 15*robot(i).R(1,j,1)]+robot(i).comPos(1,1),...
                                [0 15*robot(i).R(2,j,1)]+robot(i).comPos(1,2),...
                                [0 15*robot(i).R(3,j,1)]+robot(i).comPos(1,3),...
                                'linewidth',1.5,'color',col_ax{j});
   end
end

%%


for w = [1:60]
    wing_angles=  w;
    br = w*pi/306; bp = w*pi/180 ; by = 0;
    
    update_robot_geometry
    
    
    for i = 1:length(entities)
       for j = 1:3
           robot(i).ax_h(j).XData = [0 15*robot(i).R(1,j,counter)]+robot(i).comPos(counter,1);
           robot(i).ax_h(j).YData = [0 15*robot(i).R(2,j,counter)]+robot(i).comPos(counter,2);
           robot(i).ax_h(j).ZData = [0 15*robot(i).R(3,j,counter)]+robot(i).comPos(counter,3);                                    
       end
    end
    
    pause(0.01)

    
end


