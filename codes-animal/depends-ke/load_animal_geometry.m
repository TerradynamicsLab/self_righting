% code to setup animal geometry
load_mass_inertia;

%  animal dimensions
ani.abc = [53 22 8]/2; %mm

%sagittal plane
ani.sgp.x = 0.5*(dlc.p3d_pcut{DL.HEAD} - dlc.p3d_pcut{DL.MID}) + 0.5*(dlc.p3d_pcut{DL.MID} - dlc.p3d_pcut{DL.END}) ;
ani.sgp.x = ani.sgp.x./vecnorm(ani.sgp.x,2,2);
ani.sgp.z = repmat([0,0,1],size(ani.sgp.x,1),1);
ani.sgp.y = cross(ani.sgp.z,ani.sgp.x,2);
ani.sgp.x = cross(ani.sgp.y,ani.sgp.z,2);
ani.sgp.o = dlc.p3d_pcut{DL.MID};

% back segment
ani.back.x = dlc.p3d_pcut{DL.MID} - dlc.p3d_pcut{DL.END};
ani.back.x = ani.back.x./vecnorm(ani.back.x,2,2);
for ii=1:3
   ani.back.x(:,ii) = smooth(ani.back.x(:,ii),15); 
end
ani.back.x = ani.back.x./vecnorm(ani.back.x,2,2);
ani.back.y = cross(ani.sgp.z,ani.back.x);
ani.back.z = cross(ani.back.x,ani.back.y);

%front segment
ani.front.x = dlc.p3d_pcut{DL.HEAD} - dlc.p3d_pcut{DL.MID};
ani.front.x(:,3) = 0;
ani.front.x = ani.front.x./vecnorm(ani.front.x,2,2);
ani.front.y = cross(ani.sgp.z,ani.front.x);
ani.front.z = cross(ani.front.x,ani.front.y);

ani.front.com = ani.sgp.o + ani.front.x*ani.front.com_dx(1);
ani.back.com  = ani.sgp.o + ani.back.x*ani.back.com_dx(1);


% legs

% % find leg lengts
dlc.len.cofe.l = nanmean(vecnorm(dlc.p3d_pcut{DL.MID}-dlc.p3d_pcut{DL.LMID},2,2));
dlc.len.cofe.r = nanmean(vecnorm(dlc.p3d_pcut{DL.MID}-dlc.p3d_pcut{DL.RMID},2,2));
dlc.len.tita.l = nanmean(vecnorm(dlc.p3d_pcut{DL.LMID}-dlc.p3d_pcut{DL.LEND},2,2));
dlc.len.tita.r = nanmean(vecnorm(dlc.p3d_pcut{DL.RMID}-dlc.p3d_pcut{DL.REND},2,2));
ani.cofe.l.d_av = (dlc.len.cofe.r+dlc.len.cofe.l)/2; %16.6;
ani.cofe.r.d_av = (dlc.len.cofe.r+dlc.len.cofe.l)/2; %16.6;
ani.tita.l.d_av = (dlc.len.tita.r+dlc.len.tita.l)/2; %24;
ani.tita.r.d_av = (dlc.len.tita.r+dlc.len.tita.l)/2; %24;


% find leg axes
ani.cofe.r.x = dlc.p3d_pcut{DL.RMID} - dlc.p3d_pcut{DL.MID};
ani.cofe.l.x = dlc.p3d_pcut{DL.LMID} - dlc.p3d_pcut{DL.MID};
ani.tita.r.x = dlc.p3d_pcut{DL.REND} - dlc.p3d_pcut{DL.RMID};
ani.tita.l.x = dlc.p3d_pcut{DL.LEND} - dlc.p3d_pcut{DL.LMID};
ani.cofe.r.x = ani.cofe.r.x./vecnorm(ani.cofe.r.x,2,2);
ani.cofe.l.x = ani.cofe.l.x./vecnorm(ani.cofe.l.x,2,2);
ani.tita.r.x = ani.tita.r.x./vecnorm(ani.tita.r.x,2,2);
ani.tita.l.x = ani.tita.l.x./vecnorm(ani.tita.l.x,2,2);



ani.cofe.r.p   = ani.sgp.o + ani.cofe.r.d_av*ani.cofe.r.x;
ani.cofe.l.p   = ani.sgp.o + ani.cofe.l.d_av*ani.cofe.l.x;
ani.tita.r.p   = ani.sgp.o + ani.cofe.r.d_av*ani.cofe.r.x +   ani.tita.r.d_av*ani.tita.r.x ;
ani.tita.l.p   = ani.sgp.o + ani.cofe.l.d_av*ani.cofe.l.x +   ani.tita.l.d_av*ani.tita.l.x ;




% discretize leg rod into small segments
ani.d_cofe = [0:0.02:1]';
ani.d_tita = [0:0.01:1]';
for ii = 1:n_frames
    
   ani.sgp.R=[ani.sgp.x(ii,:)', ani.sgp.y(ii,:)', ani.sgp.z(ii,:)'];
   ani.tita.l_prel(ii,:) =  (ani.sgp.R*(ani.tita.l.p(ii,:)'-ani.sgp.o(ii,:)'));
   ani.tita.r_prel(ii,:) =  (ani.sgp.R*(ani.tita.r.p(ii,:)'-ani.sgp.o(ii,:)'));
   ani.cofe.l_prel(ii,:) =  (ani.sgp.R*(ani.cofe.l.p(ii,:)'-ani.sgp.o(ii,:)'));
   ani.cofe.r_prel(ii,:) =  (ani.sgp.R*(ani.cofe.r.p(ii,:)'-ani.sgp.o(ii,:)'));
    
   ani.cofe.r.pcl_p(:,:,ii) = ani.sgp.o(ii,:)    + ani.d_cofe .* ani.cofe.r.d_av .* ani.cofe.r.x(ii,:); 
   ani.cofe.l.pcl_p(:,:,ii) = ani.sgp.o(ii,:)    + ani.d_cofe .* ani.cofe.l.d_av .* ani.cofe.l.x(ii,:);    
   ani.tita.r.pcl_p(:,:,ii) = ani.cofe.r.p(ii,:) + ani.d_tita .* ani.tita.r.d_av .* ani.tita.r.x(ii,:); 
   ani.tita.l.pcl_p(:,:,ii) = ani.cofe.l.p(ii,:) + ani.d_tita .* ani.tita.l.d_av .* ani.tita.l.x(ii,:); 
end




