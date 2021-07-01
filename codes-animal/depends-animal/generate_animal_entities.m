% generate all the entities based on the geometry

% Surface Primitives

% Roach Body
[roach.elp.x0,roach.elp.y0, roach.elp.z0] = ellipsoid(0,0,0,roach.abc(1),roach.abc(2),roach.abc(3),roach.mesh_size);
% Roach wings
[roach.wing.elp.x0,roach.wing.elp.y0, roach.wing.elp.z0] = ellipsoid(0,0,0,roach.wing.abc(1),roach.wing.abc(2),roach.wing.abc(3),roach.mesh_size);
roach.wing.elp.lx0 =  roach.wing.elp.x0(1:roach.mesh_size/2 + 1,1:roach.mesh_size/2 + 1) - roach.wing.abc(1);
roach.wing.elp.ly0 =  roach.wing.elp.y0(1:roach.mesh_size/2 + 1,1:roach.mesh_size/2 + 1);
roach.wing.elp.lz0 =  roach.wing.elp.z0(1:roach.mesh_size/2 + 1,1:roach.mesh_size/2 + 1);
roach.wing.elp.rx0 =  roach.wing.elp.lx0;
roach.wing.elp.ry0 = -roach.wing.elp.ly0;
roach.wing.elp.rz0 =  roach.wing.elp.lz0;

roach.rpy.body = [0 0 0];
roach.rpy.wing = [0 0 0];

roach.R.body = eye(3);
roach.R.l_wing = eye(3);
roach.R.r_wing = eye(3);

roach.body.xyz = roach.R.body*[roach.elp.x0(1:end);roach.elp.y0(1:end);roach.elp.z0(1:end)];
roach.wing.l_xyz = roach.R.l_wing*[roach.wing.elp.lx0(1:end);roach.wing.elp.ly0(1:end);roach.wing.elp.lz0(1:end)] + roach.wing.l_joint;
roach.wing.r_xyz = roach.R.r_wing*[roach.wing.elp.rx0(1:end);roach.wing.elp.ry0(1:end);roach.wing.elp.rz0(1:end)] + roach.wing.r_joint;

roach.body.xx  = reshape(roach.body.xyz(1,:),   size(roach.elp.x0));
roach.body.yy  = reshape(roach.body.xyz(2,:),   size(roach.elp.x0));
roach.body.zz  = reshape(roach.body.xyz(3,:),   size(roach.elp.x0));
roach.wing.lxx = reshape(roach.wing.l_xyz(1,:), size(roach.wing.elp.lx0));
roach.wing.lyy = reshape(roach.wing.l_xyz(2,:), size(roach.wing.elp.lx0));
roach.wing.lzz = reshape(roach.wing.l_xyz(3,:), size(roach.wing.elp.lx0));
roach.wing.rxx = reshape(roach.wing.r_xyz(1,:), size(roach.wing.elp.lx0));
roach.wing.ryy = reshape(roach.wing.r_xyz(2,:), size(roach.wing.elp.lx0));
roach.wing.rzz = reshape(roach.wing.r_xyz(3,:), size(roach.wing.elp.lx0));




ff_rob = figure(2);
clf
hold on;
set(gcf,'position', [1 95 843 817]);
set(gca,'position', [-2.1856 0.0615 5.4129 0.9046]);


z_points = [];
all_points = [];
 
axis equal
 
% patch for entities
roach.surf(1) = surf(roach.body.xx,roach.body.yy,roach.body.zz,'edgealpha',0,'facecolor','r');
roach.surf(2) = surf(roach.wing.lxx,roach.wing.lyy,roach.wing.lzz,'edgealpha',0,'facecolor','b');
roach.surf(3) = surf(roach.wing.rxx,roach.wing.ryy,roach.wing.rzz,'edgealpha',0,'facecolor','b');

for i = 1:3
    roach.surf(i).FaceLighting = 'phong';
    
    z_points = [z_points  roach.surf(i).ZData(1:end)];
end
 
% com markers
%  roach(i).mh = plot3( 0, 0, 0, 'color',[0 0 1 0], 'marker', 'o','markerfacecolor','k','markeredgecolor','w','markersize',5,'linewidth',1.5); 
 
xlabel('x');ylabel('y');zlabel('z')
xticks([0]);  xticklabels({''});
yticks([0]);  yticklabels({''});
zticks([0]);  zticklabels({''});


box on

% enforce ground contact
minZ = min(z_points);
for i = [ 1 2 3] 
    roach.surf(i).ZData = roach.surf(i).ZData - minZ;
end

% ground
roach.gnd.Patch = patch([-1 -1 1 1]*40,[1 -1 -1 1]*40, [0 0 0 0],'facecolor',[1 1 1]*0.9,'facelighting','phong');
% roach(11).mh = plot3( 0, 0, 0, 'color',[0 0 1 0], 'marker', 'o','markerfacecolor',[0 0.8 0 ],'markeredgecolor','k','markersize',4,'linewidth',1.5); 


light('position',[-3 0 5]*50);
light('position',[0 -2 -5]*50);
light('position',[0 2 -5]*50);
view(158,33)
 
xlim([-30 30]);       ylim([-20 20]);       zlim([0 60]);
 

 
