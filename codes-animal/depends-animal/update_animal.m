% update the animal state based on wing angles
 
roach.R.body = EulerZYX_Fast([roach.rpy.body]*pi/180);
roach.R.l_wing = EulerZYX_Fast([ wing_angle -wing_angle 0]*pi/180);
roach.R.r_wing = EulerZYX_Fast([-wing_angle -wing_angle 0]*pi/180);

roach.body.xyz = roach.R.body*[roach.elp.x0(1:end);roach.elp.y0(1:end);roach.elp.z0(1:end)];
roach.wing.l_xyz =  roach.R.body*( roach.R.l_wing*[roach.wing.elp.lx0(1:end);roach.wing.elp.ly0(1:end);roach.wing.elp.lz0(1:end)] + roach.wing.l_joint);
roach.wing.r_xyz =  roach.R.body*( roach.R.r_wing*[roach.wing.elp.rx0(1:end);roach.wing.elp.ry0(1:end);roach.wing.elp.rz0(1:end)] + roach.wing.r_joint);

roach.body.xx  = reshape(roach.body.xyz(1,:),   size(roach.elp.x0));
roach.body.yy  = reshape(roach.body.xyz(2,:),   size(roach.elp.x0));
roach.body.zz  = reshape(roach.body.xyz(3,:),   size(roach.elp.x0));
roach.wing.lxx = reshape(roach.wing.l_xyz(1,:), size(roach.wing.elp.lx0));
roach.wing.lyy = reshape(roach.wing.l_xyz(2,:), size(roach.wing.elp.lx0));
roach.wing.lzz = reshape(roach.wing.l_xyz(3,:), size(roach.wing.elp.lx0));
roach.wing.rxx = reshape(roach.wing.r_xyz(1,:), size(roach.wing.elp.lx0));
roach.wing.ryy = reshape(roach.wing.r_xyz(2,:), size(roach.wing.elp.lx0));
roach.wing.rzz = reshape(roach.wing.r_xyz(3,:), size(roach.wing.elp.lx0));



z_points = [];
all_points = [];
 
axis equal
 
% patch for entities
roach.surf(1).XData = roach.body.xx;
roach.surf(1).YData = roach.body.yy;
roach.surf(1).ZData = roach.body.zz;

roach.surf(2).XData = roach.wing.lxx;
roach.surf(2).YData = roach.wing.lyy;
roach.surf(2).ZData = roach.wing.lzz;

roach.surf(3).XData = roach.wing.rxx;
roach.surf(3).YData = roach.wing.ryy;
roach.surf(3).ZData = roach.wing.rzz;


for ii = 1:3
    all_points = [all_points , [roach.surf(ii).XData(1:end); roach.surf(ii).YData(1:end); roach.surf(ii).ZData(1:end)] ];
end
 
% com markers
%  roach(i).mh = plot3( 0, 0, 0, 'color',[0 0 1 0], 'marker', 'o','markerfacecolor','k','markeredgecolor','w','markersize',5,'linewidth',1.5); 
 


% enforce ground contact
minZ = min(all_points(3,:));
for ii = [ 1 2 3] 
    roach.surf(ii).ZData = roach.surf(ii).ZData - minZ;
end



 
