% calculate COM position, velocity, and kinetic energy 

do_filt = 1;
mf_size = 11;


fc = 25;
fs = 2000;

[b,a] = butter(6,fc/(fs/2));

for ii=1:n_frames
    ani.sgp.R(:,:,ii)      = [ani.sgp.x(ii,:)' , ani.sgp.y(ii,:)' , ani.sgp.z(ii,:)'];
    ani.tip.l.prel(ii,:) = (ani.sgp.R(:,:,ii)'*(ani.tita.l.p(ii,:)-ani.sgp.o(ii,:))')';
    ani.tip.r.prel(ii,:) = (ani.sgp.R(:,:,ii)'*(ani.tita.r.p(ii,:)-ani.sgp.o(ii,:))')';
    
end


for ii=1:3
    ani.front.com(:,ii)       = filtfilt(b,a,ani.front.com(:,ii));
    ani.back.com(:,ii)        = filtfilt(b,a,ani.front.com(:,ii));
    
    ani.tip.r.prel(:,ii)  =filtfilt(b,a,ani.tip.r.prel(:,ii));
    ani.tip.l.prel(:,ii)  =filtfilt(b,a,ani.tip.l.prel(:,ii));

    for jj=1:size(ani.cofe.l.pcl_p,1)
        ani.cofe.l.pcl_p(jj,ii,:)    = filtfilt(b,a,squeeze(ani.cofe.l.pcl_p(jj,ii,:))); 
        ani.cofe.r.pcl_p(jj,ii,:)    = filtfilt(b,a,squeeze(ani.cofe.r.pcl_p(jj,ii,:)));
    end
    for jj=1:size(ani.tita.l.pcl_p,1)
        ani.tita.l.pcl_p(jj,ii,:)    = filtfilt(b,a,squeeze(ani.tita.l.pcl_p(jj,ii,:))); 
        ani.tita.r.pcl_p(jj,ii,:)    = filtfilt(b,a,squeeze(ani.tita.r.pcl_p(jj,ii,:)));
    end
       
end

%% com translational velocity -- spatial -- mm/s

% % body ellipsoid slices
ani.front.vs_com  = diff(ani.front.com,1,1)/timeStep;
ani.back.vs_com   = diff(ani.back.com, 1,1)/timeStep;
ani.front.vs_com  = [ani.front.vs_com; ani.front.vs_com(end-1,:)];
ani.back.vs_com   = [ani.back.vs_com ; ani.back.vs_com(end-1,:)];

% legs rods
ani.cofe.l.vs_com = [diff(ani.cofe.l.pcl_p,1,3)/timeStep;  ];   ani.cofe.l.vs_com(:,:,n_frames) = ani.cofe.l.vs_com(:,:,n_frames-1);
ani.cofe.r.vs_com = [diff(ani.cofe.r.pcl_p,1,3)/timeStep;  ];   ani.cofe.r.vs_com(:,:,n_frames) = ani.cofe.r.vs_com(:,:,n_frames-1);
ani.tita.l.vs_com = [diff(ani.tita.l.pcl_p,1,3)/timeStep;  ];   ani.tita.l.vs_com(:,:,n_frames) = ani.tita.l.vs_com(:,:,n_frames-1);
ani.tita.r.vs_com = [diff(ani.tita.r.pcl_p,1,3)/timeStep;  ];   ani.tita.r.vs_com(:,:,n_frames) = ani.tita.r.vs_com(:,:,n_frames-1);

%% com translational velocity -- body -- mm/s
for ii = 1:n_frames
    
   %  body ellipsoid slices
   ani.sgp.R(:,:,ii)      = [ani.sgp.x(ii,:)' , ani.sgp.y(ii,:)' , ani.sgp.z(ii,:)'];
   ani.front.vb_com(ii,:)  = (ani.sgp.R(:,:,ii)' * (ani.front.vs_com(ii,:)') )';
   ani.back.vb_com(ii,:)   = (ani.sgp.R(:,:,ii)'  * (ani.back.vs_com(ii,:)')  )';
   
   % legs rods -- project to the frame of back   
   ani.cofe.l.vb_com(:,:,ii) = (ani.sgp.R(:,:,ii)'  * (ani.cofe.l.vs_com(:,:,ii)')  )'; 
   ani.cofe.r.vb_com(:,:,ii) = (ani.sgp.R(:,:,ii)'  * (ani.cofe.r.vs_com(:,:,ii)')  )'; 
   ani.tita.l.vb_com(:,:,ii) = (ani.sgp.R(:,:,ii)'  * (ani.tita.l.vs_com(:,:,ii)')  )'; 
   ani.tita.r.vb_com(:,:,ii) = (ani.sgp.R(:,:,ii)'  * (ani.tita.r.vs_com(:,:,ii)')  )'; 
   
   
   
end

% leg tip -- last point in the rod point cloud
ani.tip.l.vb_com = squeeze(ani.tita.l.vb_com(end,:,:))';
ani.tip.r.vb_com = squeeze(ani.tita.r.vb_com(end,:,:))';

%% pitch velocity -- body -- rad/s

% body pitch  
ani.body_pitch  = atan2(vecnorm(cross(ani.sgp.x,ani.back.x,2),2,2),...
                                  dot(ani.sgp.x,ani.back.x,2));    
       
% find angular velocities
ani.back_omega      = [diff(ani.body_pitch) ; 0]*2000;

%%

% filter velocities
fc = 25;
fs = 2000;

[b,a] = butter(6,fc/(fs/2));

ani.back_omega = filtfilt(b,a,ani.back_omega);
for jj=1:3
  ani.front.vb_com(:,jj) = filtfilt(b,a,ani.front.vb_com(:,jj));
  ani.back.vb_com(:,jj)  = filtfilt(b,a,ani.back.vb_com(:,jj));
  
  
  for ii = 1:n_frames
    ani.cofe.l.vb_com(:,jj,ii) = filtfilt(b,a,ani.cofe.l.vb_com(:,jj,ii));
    ani.cofe.r.vb_com(:,jj,ii) = filtfilt(b,a,ani.cofe.r.vb_com(:,jj,ii));
    ani.tita.l.vb_com(:,jj,ii) = filtfilt(b,a,ani.tita.l.vb_com(:,jj,ii));    
    ani.tita.r.vb_com(:,jj,ii) = filtfilt(b,a,ani.tita.r.vb_com(:,jj,ii));
  end
  
end




%% ke components -- all in mJ

ani.front.ke.mvsq.x = 1e3* 0.5* (ani.front.m/1e3) .* (ani.front.vb_com(:,1).^2)/(1e6);
ani.front.ke.mvsq.y = 1e3* 0.5* (ani.front.m/1e3) .* (ani.front.vb_com(:,2).^2)/(1e6);
ani.front.ke.mvsq.z = 1e3* 0.5* (ani.front.m/1e3) .* (ani.front.vb_com(:,3).^2)/(1e6);

ani.back.ke.mvsq.x  = 1e3* 0.5* (ani.back.m/1e3)  .* (ani.back.vb_com(:,1).^2)/(1e6);
ani.back.ke.mvsq.y  = 1e3* 0.5* (ani.back.m/1e3)  .* (ani.back.vb_com(:,2).^2)/(1e6);
ani.back.ke.mvsq.z  = 1e3* 0.5* (ani.back.m/1e3)  .* (ani.back.vb_com(:,3).^2)/(1e6);

ani.back.ke.iwsq.y  = 1e3* 0.5* ani.back.moi(2,2).* (ani.back_omega.^2); 


for ii = 1:n_frames
    ani.cofe.l.ke.x(ii,1) = 1e3* (0.5*(ani.cofe.l.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.l.vb_com(:,1,ii).^2)/(1e6)); 
    ani.cofe.r.ke.x(ii,1) = 1e3* (0.5*(ani.cofe.r.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.r.vb_com(:,1,ii).^2)/(1e6)); 
    ani.tita.l.ke.x(ii,1) = 1e3* (0.5*(ani.tita.l.m/1e3/length(ani.d_tita)).*sum(ani.tita.l.vb_com(:,1,ii).^2)/(1e6)); 
    ani.tita.r.ke.x(ii,1) = 1e3* (0.5*(ani.tita.r.m/1e3/length(ani.d_tita)).*sum(ani.tita.r.vb_com(:,1,ii).^2)/(1e6));
    
    ani.cofe.l.ke.y(ii,1) = 1e3* (0.5*(ani.cofe.l.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.l.vb_com(:,2,ii).^2)/(1e6));
    ani.cofe.r.ke.y(ii,1) = 1e3* (0.5*(ani.cofe.r.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.r.vb_com(:,2,ii).^2)/(1e6)); 
    ani.tita.l.ke.y(ii,1) = 1e3* (0.5*(ani.tita.l.m/1e3/length(ani.d_tita)).*sum(ani.tita.l.vb_com(:,2,ii).^2)/(1e6));
    ani.tita.r.ke.y(ii,1) = 1e3* (0.5*(ani.tita.r.m/1e3/length(ani.d_tita)).*sum(ani.tita.r.vb_com(:,2,ii).^2)/(1e6));
    
    ani.cofe.l.ke.z(ii,1) = 1e3* (0.5*(ani.cofe.l.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.l.vb_com(:,3,ii).^2)/(1e6));
    ani.cofe.r.ke.z(ii,1) = 1e3* (0.5*(ani.cofe.r.m/1e3/length(ani.d_cofe)).*sum(ani.cofe.r.vb_com(:,3,ii).^2)/(1e6));
    ani.tita.l.ke.z(ii,1) = 1e3* (0.5*(ani.tita.l.m/1e3/length(ani.d_tita)).*sum(ani.tita.l.vb_com(:,3,ii).^2)/(1e6));
    ani.tita.r.ke.z(ii,1) = 1e3* (0.5*(ani.tita.r.m/1e3/length(ani.d_tita)).*sum(ani.tita.r.vb_com(:,3,ii).^2)/(1e6));
end

ani.tip.l.ke.x = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.l.vb_com(:,1).^2)/(1e6)); 
ani.tip.r.ke.x = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.r.vb_com(:,1).^2)/(1e6)); 

ani.tip.l.ke.y = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.l.vb_com(:,2).^2)/(1e6)); 
ani.tip.r.ke.y = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.r.vb_com(:,2).^2)/(1e6)); 

ani.tip.l.ke.z = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.l.vb_com(:,3).^2)/(1e6)); 
ani.tip.r.ke.z = 1e3* (0.5*(ani.ball.m/1e3).*   (ani.tip.r.vb_com(:,3).^2)/(1e6)); 


%%

ani.ke.pitch.body = ani.front.ke.mvsq.x + ani.back.ke.mvsq.x + ... 
                    ani.back.ke.iwsq.y ;

ani.ke.pitch.leg  = ani.cofe.l.ke.x     + ani.tita.l.ke.x    + ...
                    ani.cofe.r.ke.x     + ani.tita.r.ke.x    + ...
                    ani.tip.l.ke.x      + ani.tip.r.ke.x     ;

ani.ke.roll.body  =  ani.front.ke.mvsq.y + ani.back.ke.mvsq.y ;

ani.ke.roll.leg   = ani.cofe.l.ke.y     + ani.tita.l.ke.y    + ...
                    ani.cofe.r.ke.y     + ani.tita.r.ke.y    + ...
                    ani.tip.l.ke.y      + ani.tip.r.ke.y     ;

ani.ke.pitch.all  = ani.ke.pitch.body + ani.ke.pitch.leg;
ani.ke.roll.all   = ani.ke.roll.body  + ani.ke.roll.leg;

ke.p.body = smooth(medfilt1(fillmissing(ani.ke.pitch.body,'linear','endvalues','nearest'),15),15);   
ke.p.leg  = smooth(medfilt1(fillmissing(ani.ke.pitch.leg, 'linear','endvalues','nearest'),15),15);   
ke.p.all  = smooth(medfilt1(fillmissing(ani.ke.pitch.leg + ani.ke.pitch.body, 'linear','endvalues','nearest'),15),15);   
ke.r.body = smooth(medfilt1(fillmissing(ani.ke.roll.body, 'linear','endvalues','nearest'),15),15);   
ke.r.leg  = smooth(medfilt1(fillmissing(ani.ke.roll.leg,  'linear','endvalues','nearest'),15),15);  
ke.r.all  = smooth(medfilt1(fillmissing(ani.ke.roll.leg + ani.ke.roll.body, 'linear','endvalues','nearest'),15),15);