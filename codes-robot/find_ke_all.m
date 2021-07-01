% code to estimate the ke components


clear
close all

addpath(genpath('ke-depends'))

% Load the dependencies
addpath('pel-depends');

load ..\data-robot\barrier.mat

% read the robot data
robot_data = xlsread('..\data-robot\robotdata.xlsx');
robot_data = robot_data(:,5);
robot_data(robot_data==0)=10;

% assemble robot model from individual entities
robot = struct();
load_robot_model;


ke_data = struct();

f_list = dir(['..\data-robot\raw_data\*bag*']);

% loop over all the trials
    
for bag = 1:length(f_list)
   %% load input data


   % trial name 
   f_name = f_list(bag).name


   ke_data(bag).trial_name = f_name;

   if(mod(bag,5))
        ke_data(bag).trial_id = mod(bag,5);
   else
        ke_data(bag).trial_id = 5;
   end

   ke_data(bag).wing_angle = str2num(f_name(6:7));
   temp_vib_angle = str2num(f_name(18:19));   
   if isempty(temp_vib_angle)
       temp_vib_angle = 0;
   end
    switch temp_vib_angle           
       case 0            
           ke_data(bag).vib_angle = 0;
       case 15 
           ke_data(bag).vib_angle = 15;
       case 30 
           ke_data(bag).vib_angle = 30;
       case 45 
           ke_data(bag).vib_angle = 45;        
    end



    % Load recorded robot trajectory
    [robot_quat_list, robot_angle_rect, vib_angle_rect , eul_list_1,eul_list_2] = load_robot_trajectory(f_name);

    eul_list_1 = eul_list_1'; eul_list_1(:,2) = -eul_list_1(:,2);
    eul_list_2 = eul_list_2'; eul_list_2(:,2) = -eul_list_2(:,2);

    % Find when the first wing opening is by matching quaternion time and wing time
    tmp = abs(robot_quat_list(:,1)-min(robot_angle_rect(:,1)));
    [~, wing_open_idx] = min(tmp);    

    tmp = abs(robot_quat_list(:,1)-min(vib_angle_rect(:,1)));
    [~, vib_start_idx] = min(tmp);
    % all time frames are w.r.t IMU data
    wing_open_time = robot_quat_list(wing_open_idx,1);
    wing_close_time = wing_open_time + 1;
    wing_open_time_2 = wing_open_time + 2;
    vib_start_time = robot_quat_list(vib_start_idx,1);
    data_end_time = robot_quat_list(end,1);

    tmp = abs(robot_quat_list(:,1)-wing_close_time);
    [~, wing_close_idx] = min(tmp);
    tmp = abs(robot_quat_list(:,1)-wing_open_time_2);
    [~, wing_open_idx_2] = min(tmp);



    fps_real = size(robot_quat_list,1)/max(max(robot_quat_list(:,1)));

    if(vib_start_idx==wing_open_idx)
        wing_open_idx = wing_open_idx + 1;
    end           



    nFrames = size(robot_quat_list,1)-vib_start_idx;
    for i = 1:10
        robot(i).comPos = nan(nFrames,3);
        robot(i).R      = nan(3,3,nFrames);    
        robot(i).eul    = nan(nFrames,3);    
    end

    % robot
    generate_robot_entities;
    counter = 0;

    % data structures for pointcloud
    pcl = struct();
    initialise_pcl_ds;

    % geometry update -- loop over all the points
    disp(['updating geometry : trial ' num2str(bag) ])
    for i  = vib_start_idx:size(robot_quat_list,1)
        ke_robot_geometry_update;
    end


% calculate velocities
disp(['updating velocity : trial ' num2str(bag) ])
ke_velocity_updates

% calculate ke
disp(['updating energy : trial ' num2str(bag) ])
ke_energy_updates


clearvars t_output

save ../data-robot/ke_all.mat ke_data

end

