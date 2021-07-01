% calculates from start idx to end of attempt
t_input = 0:(size(robot(M5).comPos,1)-1); 
t_input = t_input/fps_real;
dt = 0.01;
t_output = t_input(1):dt:t_input(end);

% store the timing information
ke_data(bag).t_input = t_input;
ke_data(bag).t_output = t_output;
ke_data(bag).t_vib_start = t_input(1);
ke_data(bag).t_wing_open = t_input(wing_open_idx - vib_start_idx);
ke_data(bag).t_wing_close = t_input(wing_close_idx - vib_start_idx);



%% m5 + pend
% for M5 and PEND, we express all entities w.r.t M5 body frame
for ob = [M5 PEN]
    
    % interpolate the positions and euler angles    
    ke_data(bag).robot(ob).comPos_itp(:,1) = interp1(t_input,  robot(ob).comPos(:,1), t_output);
    ke_data(bag).robot(ob).comPos_itp(:,2) = interp1(t_input,  robot(ob).comPos(:,2), t_output);
    ke_data(bag).robot(ob).comPos_itp(:,3) = interp1(t_input,  robot(ob).comPos(:,3), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,1) = interp1(t_input,  robot(ob).eul(:,1), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,2) = interp1(t_input,  robot(ob).eul(:,2), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,3) = interp1(t_input,  robot(ob).eul(:,3), t_output);
    % NOTE - there are better ways to interpolate Euler angles
    
    % get rotation matrices for interpolated euler angles
    for i = 1:size(ke_data(bag).robot(ob).comEul_itp(:,1))
        ke_data(bag).robot(ob).R(:,:,i) = EulerZYX_Fast(ke_data(bag).robot(ob).comEul_itp(i,:));
    end
    
    % OB - V - world - mm/s
    ke_data(bag).robot(ob).vel(:,1) = [diff(ke_data(bag).robot(ob).comPos_itp(:,1)); nan ]/dt;
    ke_data(bag).robot(ob).vel(:,2) = [diff(ke_data(bag).robot(ob).comPos_itp(:,2)); nan ]/dt;
    ke_data(bag).robot(ob).vel(:,3) = [diff(ke_data(bag).robot(ob).comPos_itp(:,3)); nan ]/dt;
            
    % OB - V - M5 - mm/s
    % we use M5's rotation here -- M5 and PEN will be seen in M5's frame
    for i = 1:size(ke_data(bag).robot(ob).vel,1)
        ke_data(bag).robot(ob).vel_b(i,:) = ( ke_data(bag).robot(M5).R(:,:,i)'*ke_data(bag).robot(ob).vel(i,:)')';        
    end
    
    % OB - W - M5 - rad/s
    for i  = 1:size(ke_data(bag).robot(ob).comEul_itp,1)-1
       ke_data(bag).robot(ob).omega_hat(:,:,i) = (ke_data(bag).robot(ob).R(:,:,i)')*...
                                                   [   ke_data(bag).robot(ob).R(:,:,i+1) - ...
                                                       ke_data(bag).robot(ob).R(:,:,i)     ...
                                                   ]/dt;
       ke_data(bag).robot(ob).omega_rot(i,:) = [    ke_data(bag).robot(ob).omega_hat(3,2,i) - ke_data(bag).robot(ob).omega_hat(2,3,i),...
                                                    ke_data(bag).robot(ob).omega_hat(1,3,i) - ke_data(bag).robot(ob).omega_hat(3,1,i),...,...
                                                    ke_data(bag).robot(ob).omega_hat(2,1,i) - ke_data(bag).robot(ob).omega_hat(1,2,i)];       
       % for pendulum, shift to M5 frame
       if(ob==PEN)
        ke_data(bag).robot(ob).omega_rot(i,:) = (ke_data(bag).robot(M5).R(:,:,i)'*ke_data(bag).robot(ob).omega_rot(i,:)')';
       end
    end
    ke_data(bag).robot(ob).omega_rot(i+1,:) = [nan nan nan];
end


%% head/wings pointcloud 

% Head - V - world - mm/s
for i = 1:size(pcl.head.x,2)
    pcl.v_world.head.x(:,i) = [diff(interp1(t_input, pcl.head.x(:,i),t_output)),nan]'/dt;
    pcl.v_world.head.y(:,i) = [diff(interp1(t_input, pcl.head.y(:,i),t_output)),nan]'/dt;
    pcl.v_world.head.z(:,i) = [diff(interp1(t_input, pcl.head.z(:,i),t_output)),nan]'/dt;
end
% W1 - V - world - mm/s
for i = 1:size(pcl.w1.x,2)
    pcl.v_world.w1.x(:,i) = [diff(interp1(t_input, pcl.w1.x(:,i),t_output)),nan]'/dt;
    pcl.v_world.w1.y(:,i) = [diff(interp1(t_input, pcl.w1.y(:,i),t_output)),nan]'/dt;
    pcl.v_world.w1.z(:,i) = [diff(interp1(t_input, pcl.w1.z(:,i),t_output)),nan]'/dt;
end
% W2 - V - world - mm/s
for i = 1:size(pcl.w2.x,2)
    pcl.v_world.w2.x(:,i) = [diff(interp1(t_input, pcl.w2.x(:,i),t_output)),nan]'/dt;
    pcl.v_world.w2.y(:,i) = [diff(interp1(t_input, pcl.w2.y(:,i),t_output)),nan]'/dt;
    pcl.v_world.w2.z(:,i) = [diff(interp1(t_input, pcl.w2.z(:,i),t_output)),nan]'/dt;
end

% loop:each instant in time and shift to M5's frame
for i = 1:size(ke_data(bag).robot(M5).comEul_itp(:,1))
    % HEAD - V - M5 - mm/s
    tmp = ke_data(bag).robot(M5).R(:,:,i)'* [pcl.v_world.head.x(i,:) ; pcl.v_world.head.y(i,:);pcl.v_world.head.z(i,:)];
    pcl.v_body.head.x(i,:) = tmp(1,:);
    pcl.v_body.head.y(i,:) = tmp(2,:);
    pcl.v_body.head.z(i,:) = tmp(3,:);
    % W1 - V - M5 - mm/s
    tmp = ke_data(bag).robot(M5).R(:,:,i)'* [pcl.v_world.w1.x(i,:) ; pcl.v_world.w1.y(i,:);pcl.v_world.w1.z(i,:)];
    pcl.v_body.w1.x(i,:) = tmp(1,:);
    pcl.v_body.w1.y(i,:) = tmp(2,:);
    pcl.v_body.w1.z(i,:) = tmp(3,:);
    % W2 - V - M5 - mm/s
    tmp = ke_data(bag).robot(M5).R(:,:,i)'* [pcl.v_world.w2.x(i,:) ; pcl.v_world.w2.y(i,:);pcl.v_world.w2.z(i,:)];
    pcl.v_body.w2.x(i,:) = tmp(1,:);
    pcl.v_body.w2.y(i,:) = tmp(2,:);
    pcl.v_body.w2.z(i,:) = tmp(3,:);    
end


%% motors and tail rod
for ob = [M1 M2 M3 M4 TAIL]
    
    % interpolate the positions and euler angles    
    ke_data(bag).robot(ob).comPos_itp(:,1) = interp1(t_input,  robot(ob).comPos(:,1), t_output);
    ke_data(bag).robot(ob).comPos_itp(:,2) = interp1(t_input,  robot(ob).comPos(:,2), t_output);
    ke_data(bag).robot(ob).comPos_itp(:,3) = interp1(t_input,  robot(ob).comPos(:,3), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,1) = interp1(t_input,  robot(ob).eul(:,1), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,2) = interp1(t_input,  robot(ob).eul(:,2), t_output);
    ke_data(bag).robot(ob).comEul_itp(:,3) = interp1(t_input,  robot(ob).eul(:,3), t_output);
    % NOTE - there are better ways to interpolate Euler angles
    
    % get rotation matrices for interpolated euler angles
    for i = 1:size(ke_data(bag).robot(ob).comEul_itp(:,1))
        ke_data(bag).robot(ob).R(:,:,i) = EulerZYX_Fast(ke_data(bag).robot(ob).comEul_itp(i,:));
    end
    
    % OB - V - world - mm/s
    ke_data(bag).robot(ob).vel(:,1) = [diff(ke_data(bag).robot(ob).comPos_itp(:,1)); nan ]/dt;
    ke_data(bag).robot(ob).vel(:,2) = [diff(ke_data(bag).robot(ob).comPos_itp(:,2)); nan ]/dt;
    ke_data(bag).robot(ob).vel(:,3) = [diff(ke_data(bag).robot(ob).comPos_itp(:,3)); nan ]/dt;
            
    % OB - V - body - mm/s
    for i = 1:size(ke_data(bag).robot(ob).vel,1)
        ke_data(bag).robot(ob).vel_b(i,:) = ( ke_data(bag).robot(M5).R(:,:,i)'*ke_data(bag).robot(ob).vel(i,:)')';        
    end
    
    % OB - W - body - rad/s
    for i  = 1:size(ke_data(bag).robot(ob).comEul_itp,1)-1
       ke_data(bag).robot(ob).omega_hat(:,:,i) = (ke_data(bag).robot(M5).R(:,:,i)')*...
                                                   [   ke_data(bag).robot(ob).R(:,:,i+1) - ...
                                                       ke_data(bag).robot(ob).R(:,:,i)     ...
                                                   ]/dt;
       ke_data(bag).robot(ob).omega_rot(i,:) = [    ke_data(bag).robot(ob).omega_hat(3,2,i) - ke_data(bag).robot(ob).omega_hat(2,3,i),...
                                                    ke_data(bag).robot(ob).omega_hat(1,3,i) - ke_data(bag).robot(ob).omega_hat(3,1,i),...,...
                                                    ke_data(bag).robot(ob).omega_hat(2,1,i) - ke_data(bag).robot(ob).omega_hat(1,2,i)];
    end
    ke_data(bag).robot(ob).omega_rot(i+1,:) = [nan nan nan];
end


