% code to calculate the landscape 

clear
close all

addpath(genpath('pel-depends'))

pelSteps = 1;

% Load and build the robot from parts
load_robot_model;

wing_angle_list= -10:.5:90;

%% assign varibles
surf_data = cell(1,numel(wing_angle_list));
xp = nan*meshgrid(-180:pelSteps:180);
yp = xp;
zp = xp;

for counter = 1:numel(wing_angle_list)
    surf_data{counter}.XData = meshgrid(-180:pelSteps:180);
    surf_data{counter}.YData = meshgrid(-180:pelSteps:180);
    surf_data{counter}.ZData = meshgrid(-180:pelSteps:180);
    surf_data{counter}.wa    = wing_angle_list(counter)
    surf_data{counter}.robot = struct();
end

%% generate geoemtry and graphics
 
%pel
ff_pel = figure(100);
clf
set(gcf,'position', [854 96 600 500]);
hold on;box on; set(gca,'fontsize',15)
[xdummy,ydummy] = meshgrid(1:5,1:5);
ff.sh_pel = surf(xdummy,ydummy,0*xdummy, 'FaceAlpha' , 1, 'edgealpha', 0);
zff.mh_state = plot3( 0, 0, 0, 'color',[0 0 1 0], 'marker', 'o','markerfacecolor',[0 0.7 0],'markeredgecolor','k','markersize',10,'linewidth',1.5); 
ff.ph_traj = plot(0,0, 'color', 'w','linewidth',2);
colormap(gca,fire); 
cbar = colorbar();
axis equal
view(-270,90)
xlim([-180 180])
ylim([-180 180])


% make data structures to store position orientation data
nFrames = length(wing_angle_list);
for i = 1:10
    robot(i).comPos = nan(nFrames,3);
    robot(i).R      = nan(3,3,nFrames);    
    robot(i).eul    = nan(nFrames,3);    
end

%robot    
generate_robot_entities;
counter = 0;


% loop over all the points
for i  = 1:length(wing_angle_list) %+ 100 %size(robot_quat_list,1)

   
  
    % inputs for robot geometry update
    wing_angles(1)  = wing_angle_list(i);                         % wing_pitch
    wing_angles(2)  = wing_angle_list(i);                         % wing_roll
    vibration_angle = 0;    % vibration
    br = 0; bp = 0 ; by = 0;                                      % body RPY

    % call update
    update_robot_geometry;        

    % Call landscape
    [xp yp zp] = righting_energy_landscape(all_points(1,:),all_points(2,:),all_points(3,:),center_of_mass , pelSteps,pelSteps);

    % update data structyres
    surf_data{i}.XData = xp;
    surf_data{i}.YData = yp;
    surf_data{i}.ZData = zp*sum(mass)*9.81/1000;  % Energy in mJ;
    surf_data{i}.wing_roll_r = roll_ang_R;
    surf_data{i}.wing_roll_l = roll_ang_L;
    surf_data{i}.wing_pitch_r = pitch_ang_R;
    surf_data{i}.wing_pitch_l = pitch_ang_L;
    surf_data{i}.robot = robot;


   
   ff.sh_pel.XData = surf_data{i}.XData;
   ff.sh_pel.YData = surf_data{i}.YData;
   ff.sh_pel.ZData = surf_data{i}.ZData;



end
save(['../data-robot/landscape_full_wing_range.mat'], 'surf_data');

close all

