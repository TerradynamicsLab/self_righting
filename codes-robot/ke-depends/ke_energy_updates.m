% update ke values based on the velocities


% Mass : g to kg     (divide by 1000)
% Inertia : already in kg.m*m
% Vel : mm/s to m/s (divide by 1000*1000)
% Avel : already in rad/s 
% KE : Finally to mJ (multiply by 1000)

%% ke components for the solid parts -- all in mJ
for ob = [M1 M2 M3 M4 M5 PEN TAIL]
    
    ke_data(bag).robot(ob).ke_mvsq_x = 0.5*(mass(ob)/1000)     .*(ke_data(bag).robot(ob).vel_b(:,1).^2)*1000/(1000*1000) ; 
    ke_data(bag).robot(ob).ke_mvsq_y = 0.5*(mass(ob)/1000)     .*(ke_data(bag).robot(ob).vel_b(:,2).^2)*1000/(1000*1000) ; 
    ke_data(bag).robot(ob).ke_mvsq_z = 0.5*(mass(ob)/1000)     .*(ke_data(bag).robot(ob).vel_b(:,3).^2)*1000/(1000*1000) ; 
    ke_data(bag).robot(ob).ke_iwsq_x = 0.5*(robot(ob).Ibb(1,1)).*(ke_data(bag).robot(ob).omega_rot(:,1).^2)*1000; 
    ke_data(bag).robot(ob).ke_iwsq_y = 0.5*(robot(ob).Ibb(2,2)).*(ke_data(bag).robot(ob).omega_rot(:,2).^2)*1000; 
    ke_data(bag).robot(ob).ke_iwsq_z = 0.5*(robot(ob).Ibb(3,3)).*(ke_data(bag).robot(ob).omega_rot(:,3).^2)*1000; 
end


%% ke components for pointclouds -- all in mJ
n_head = length(robot(HEAD).Patch.Vertices(:,1));
n_w1   = length(robot(W1).Patch.Vertices(:,1));
n_w2   = length(robot(W2).Patch.Vertices(:,1));

ke_data(bag).robot(HEAD).ke_mvsq_x  = 0.5*(mass(ob)/n_head/1000).*sum(pcl.v_body.head.x.^2,2)*1000/(1000*1000);
ke_data(bag).robot(HEAD).ke_mvsq_y  = 0.5*(mass(ob)/n_head/1000).*sum(pcl.v_body.head.y.^2,2)*1000/(1000*1000);
ke_data(bag).robot(HEAD).ke_mvsq_z  = 0.5*(mass(ob)/n_head/1000).*sum(pcl.v_body.head.z.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W1).ke_mvsq_x    = 0.5*(mass(ob)/n_w1/1000).*sum(pcl.v_body.w1.x.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W1).ke_mvsq_y    = 0.5*(mass(ob)/n_w1/1000).*sum(pcl.v_body.w1.y.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W1).ke_mvsq_z    = 0.5*(mass(ob)/n_w1/1000).*sum(pcl.v_body.w1.z.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W2).ke_mvsq_x    = 0.5*(mass(ob)/n_w2/1000).*sum(pcl.v_body.w2.x.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W2).ke_mvsq_y    = 0.5*(mass(ob)/n_w2/1000).*sum(pcl.v_body.w2.y.^2,2)*1000/(1000*1000);
ke_data(bag).robot(W2).ke_mvsq_z    = 0.5*(mass(ob)/n_w2/1000).*sum(pcl.v_body.w2.z.^2,2)*1000/(1000*1000);
% since these are pointclouds, they have no rotational components
for ob = [HEAD W1 W2]    
    ke_data(bag).robot(ob).ke_iwsq_x = ke_data(bag).robot(ob).ke_mvsq_z * 0; 
    ke_data(bag).robot(ob).ke_iwsq_y = ke_data(bag).robot(ob).ke_mvsq_z * 0; 
    ke_data(bag).robot(ob).ke_iwsq_z = ke_data(bag).robot(ob).ke_mvsq_z * 0; 
end


%% filter

fc = 10;
fs = 100;
[b ,a] = butter(6, fc/(fs/2));


ke_data(bag).ke_sag_t = 0;
ke_data(bag).ke_oth_t = 0;
ke_data(bag).ke_sag_r = 0;
ke_data(bag).ke_oth_r = 0;
ke_data(bag).ke_sag = 0;
ke_data(bag).ke_oth = 0;
ke_data(bag).kef_sag_t = 0;
ke_data(bag).kef_oth_t = 0;
ke_data(bag).kef_sag_r = 0;
ke_data(bag).kef_oth_r = 0;
ke_data(bag).kef_sag = 0;
ke_data(bag).kef_oth = 0;

for ob = [ M1 M2 M3 M4 M5 HEAD W1 W2 PEN TAIL ]

     % store the components for each entity
     ke_data(bag).robot(ob).ke_sag_t =  ke_data(bag).robot(ob).ke_mvsq_x + ...
                                      0*ke_data(bag).robot(ob).ke_mvsq_z;         
     ke_data(bag).robot(ob).ke_oth_t =  ke_data(bag).robot(ob).ke_mvsq_y;         
     ke_data(bag).robot(ob).ke_sag_r =  ke_data(bag).robot(ob).ke_iwsq_y;                                  
     ke_data(bag).robot(ob).ke_oth_r =  ke_data(bag).robot(ob).ke_iwsq_x + ...
                                      0*ke_data(bag).robot(ob).ke_iwsq_z;

     % store the total from each entity                                          
     ke_data(bag).ke_sag_t =  ke_data(bag).ke_sag_t  + ke_data(bag).robot(ob).ke_sag_t;                                  
     ke_data(bag).ke_oth_t =  ke_data(bag).ke_oth_t  + ke_data(bag).robot(ob).ke_oth_t;                                                                    
     ke_data(bag).ke_sag_r =  ke_data(bag).ke_sag_r  + ke_data(bag).robot(ob).ke_sag_r;                                                                                                     
     ke_data(bag).ke_oth_r =  ke_data(bag).ke_oth_r  + ke_data(bag).robot(ob).ke_oth_r;                                                                                                                                       
     ke_data(bag).ke_sag = ke_data(bag).ke_sag_r + ke_data(bag).ke_sag_t;
     ke_data(bag).ke_oth = ke_data(bag).ke_oth_r + ke_data(bag).ke_oth_t;   

     % filter the data
     idx_nan = find(isnan(ke_data(bag).robot(ob).ke_sag_t));   
     if(t_output(idx_nan(1))<(robot_data(bag)-0.1))
          disp('umm')
     end
     ke_data(bag).robot(ob).ke_sag_t(idx_nan) = 0;
     idx_nan = find(isnan(ke_data(bag).robot(ob).ke_sag_r));         
     ke_data(bag).robot(ob).ke_sag_r(idx_nan) = 0;
     idx_nan = find(isnan(ke_data(bag).robot(ob).ke_oth_t));         
     ke_data(bag).robot(ob).ke_oth_t(idx_nan) = 0;
     idx_nan = find(isnan(ke_data(bag).robot(ob).ke_oth_r));         
     ke_data(bag).robot(ob).ke_oth_r(idx_nan) = 0;

     ke_data(bag).robot(ob).kef_sag_t =  filtfilt(b,a,[ke_data(bag).robot(ob).ke_sag_t(1:end-1);0] );
     ke_data(bag).robot(ob).kef_oth_t =  filtfilt(b,a,[ke_data(bag).robot(ob).ke_oth_t(1:end-1);0] );
     ke_data(bag).robot(ob).kef_sag_r =  filtfilt(b,a,[ke_data(bag).robot(ob).ke_sag_r(1:end-1);0] );
     ke_data(bag).robot(ob).kef_oth_r =  filtfilt(b,a,[ke_data(bag).robot(ob).ke_oth_r(1:end-1);0] );

     % store the total from each entity                                          
     ke_data(bag).kef_sag_t =  ke_data(bag).kef_sag_t  + ke_data(bag).robot(ob).kef_sag_t;                                  
     ke_data(bag).kef_oth_t =  ke_data(bag).kef_oth_t  + ke_data(bag).robot(ob).kef_oth_t;                                                                    
     ke_data(bag).kef_sag_r =  ke_data(bag).kef_sag_r  + ke_data(bag).robot(ob).kef_sag_r;                                                                                                     
     ke_data(bag).kef_oth_r =  ke_data(bag).kef_oth_r  + ke_data(bag).robot(ob).kef_oth_r;                                                                                                                                       
     ke_data(bag).kef_sag = ke_data(bag).kef_sag_r + ke_data(bag).kef_sag_t;
     ke_data(bag).kef_oth = ke_data(bag).kef_oth_r + ke_data(bag).kef_oth_t;   
     
     if(any( [ sum(isnan(ke_data(bag).robot(ob).kef_sag_t)), ...
               sum(isnan(ke_data(bag).robot(ob).kef_sag_r)), ...
               sum(isnan(ke_data(bag).robot(ob).kef_oth_t)), ...
               sum(isnan(ke_data(bag).robot(ob).kef_oth_r)) ]))
            fid = fopen( ['ke-depends\' num2str(bag) '.txt'], 'wt' );
            fclose(fid)
         
     end

end


1;