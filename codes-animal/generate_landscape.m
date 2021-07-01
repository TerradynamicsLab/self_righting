% code to generate animal landscape for self-righting

clear
close all

addpath(genpath('depends-animal'))

% grid size for roll-pitch space
pel_step=0.5;

% range of wing angles fro which landscape is calculated
wing_angle_list = 0:.5:90;


% animal body dimension - [2a 2b 2c] 
roach.abc = [53 22 8]/2; %mm
roach.m = 2.6; %g
roach.com = [0 0 0]'; % mm

% animal wing geometry
roach.wing.abc = [0.8*53 23 0.4*8]/2; %mm
roach.wing.l_joint = [.75*roach.abc(1) -2 0 ]'; 
roach.wing.l_joint(3) = -roach.abc(3)*real(sqrt(1 - (roach.wing.l_joint(1)^2/roach.abc(1)^2) - (roach.wing.l_joint(2)^2/roach.abc(2)^2)));
roach.wing.r_joint = roach.wing.l_joint.*[1 -1 1]';
roach.mesh_size = 50;

% generate geometry and graphics entities
generate_animal_entities


% allocate data structures for landscape
[pitch_mesh, roll_mesh] = meshgrid((-180:pel_step:180)*pi/180, (-180:pel_step:180)*pi/180);
pitch_mesh = pitch_mesh(:);
roll_mesh  = roll_mesh(:);
% we need only the bottom row of the rotation matrix to find the z-coordinate for all body roll/pitch 
% find the rotation matrix for all points on body pitch-roll space

for i=1:length(wing_angle_list)
    
    % inputs for animal geometry
    wing_angle = wing_angle_list(i);

    % update animal geometry
    update_animal
    counter = 1;
    for p = 1:numel(pitch_mesh)
%         for r = 1:numel(roll_mesh)
        % find rotated body for all points in the body pitch-roll grid
        R_mesh = [-sin(pitch_mesh(p))   cos(pitch_mesh(p)).*sin(roll_mesh(p))  cos(pitch_mesh(p)).*cos(roll_mesh(p))];
        rotated_points = R_mesh*all_points;
        % find com height for all points in the body pitch-roll grid (with ground contact constraint) 
        relative_com_height{i}(counter) = -min(rotated_points);%,[],2);
        counter = counter+1
%         end
    end
    i
    
    drawnow

end

save ../data-animal/animal_pel.mat roach wing_angle_list pitch_mesh roll_mesh R_mesh relative_com_height