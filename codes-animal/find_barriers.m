% code to find the energy barriers for animal landscape

clear
make_videos = 0;
load ../data-animal/animal_pel.mat


%% landscape figure
ff_pel = figure(1);
clf
colormap(firefly)
set(gcf,'position', [854 96 500 500]);
hold on;box on; set(gca,'fontsize',15);

mesh_size = sqrt(numel(roll_mesh));
roll_mesh  = reshape(roll_mesh, mesh_size,mesh_size);
pitch_mesh = reshape(pitch_mesh, mesh_size,mesh_size);

ff.sh_pel = surf(roll_mesh*180/pi,pitch_mesh*180/pi,reshape(relative_com_height{1},size(roll_mesh)), 'FaceAlpha' , 1, 'edgealpha', 0);
ff.mh_state = plot3( 0, 0, 0, 'color',[0 0 1 0], 'marker', 'o','markerfacecolor',[0 0.7 0],'markeredgecolor','k','markersize',10,'linewidth',1.5); 
ff.ph_traj = plot(0,0, 'color', 'w','linewidth',2);

cbar = colorbar();
axis equal
xlim([-180 180])
ylim([-90 180])
view(-120,60)

set(gca,'outerposition',[0 0 1 1 ]);
axis normal
set(gca,'plotboxaspectratio', [1.8 1.8 1.2])
xticks([-2 -1 0 1 2]*90);  xticklabels({''});
yticks([-2 -1 0 1 2]*90);  yticklabels({''});
zticks([0 25 50]);  zticklabels({''});
zlim([0 50])
set(gca,'fontsize',15, 'position',[-0.032 0.02 0.95 0.95]);
% caxis([40 420])

cbar.Position = [0.9 0.2 0.025 0.6];

%% add code to setup barrier plots
figure(2)
clf;set(gcf,'position' , [1454 96 500 500]);
hold on ; box on ; 
set(gca,'fontsize' , 15,'position',[0 0 1 1]);    
%plot elements
tt = 0:0.01:2*pi;
pel.b_edge = plot(nan,nan,'k','linewidth',2)
pel.b_fill = fill(0,0,'k')';
set(get(gca,'xaxis'),'visible','off')
set(get(gca,'yaxis'),'visible','off','direction','reverse')

axis equal
xlim([-1 1]*40);ylim([-1 1]*40)
hh = hline(0, 'k'); set(hh,'color',[0 0 0 0.3]);
hh = vline(0, 'k'); set(hh,'color',[0 0 0 0.3]);
    


%% code to calculate barriers
valid_wing_max = 142; %no saddle beyond this wing opening angles
for i = 1:valid_wing_max
   
       ff.sh_pel.ZData = reshape(relative_com_height{i},size(roll_mesh));

       % make the surface interpolant for further calculation-- NOTE
       % X=roll, y=pitch
       pel.itp = griddedInterpolant(ff.sh_pel.XData,ff.sh_pel.YData,ff.sh_pel.ZData);

       % find pitch minima

       valid_idx = find((ff.sh_pel.YData>-5) & (ff.sh_pel.YData<90) & (ff.sh_pel.XData==0) );
       valid_pitch = ff.sh_pel.YData(valid_idx);
       valid_zcom = ff.sh_pel.ZData(valid_idx);
       [min_val, min_idx] = min(valid_zcom);
       minima.pitch.p(i) = valid_pitch(min_idx);
       minima.pitch.r(i) = 0;
       minima.pitch.e(i) = min_val;


       % add the minima markers   
       ff.mh_state.XData = 0;
       ff.mh_state.YData = minima.pitch.p(i);
       ff.mh_state.ZData = minima.pitch.e(i) + 2;

       % Start ray tracing from average points         
       angle_step = 400;   % this is a temp variable used for finding boundaries on the the landscape
       angle_incr = sign(minima.pitch.r(i))*0.01;
       if(angle_incr ==0), angle_incr = 0.01;end
       end_points = [linspace( minima.pitch.r(i), -179.99                      ,angle_step)',linspace( -180, -180   ,angle_step)' ;...                 
                     linspace(-180              , -180                         ,angle_step)',linspace( -180,  179.99,angle_step)' ;...
                     linspace(-180              ,  179.99                      ,angle_step)',linspace(  180,  180   ,angle_step)' ;...
                     linspace( 180              ,  180                         ,angle_step)',linspace(  180, -179.99,angle_step)' ;...
                     linspace( 180              , minima.pitch.r(i)-angle_incr ,angle_step)',linspace( -180, -180   ,angle_step)' ];

     end_points(:,3) = wrapTo360(  atan2(  end_points(:,2)-minima.pitch.p(i), ...
                                           end_points(:,1)-minima.pitch.r(i)  ) ...
                                  *180/pi);
     end_points = sortrows(end_points,3);

     % Remove duplicate points
     [aa bb cc] = unique(end_points(:,3));
     end_points = end_points(bb,:);


     % Get section of the landscape from average point to the end
     barrier_points = [];

     swept_angles = wrapTo360(linspace(0,360,500)); % how many points in the angular sweep
     sec_steps = 100; % how many data points in a section

     % radial sweep
        for j = 1:length(swept_angles)

              section_roll  = linspace(minima.pitch.r(i),interp1(end_points(:,3),end_points(:,1),swept_angles(j)),sec_steps);
              section_pitch = linspace(minima.pitch.p(i),interp1(end_points(:,3),end_points(:,2),swept_angles(j)),sec_steps);
              section_energy = pel.itp(section_roll ,section_pitch);

              [pks ,locs ,wid,prom] = findpeaks(section_energy);

              %choose only prominent peaks
              if(~isempty(prom))
                  pks  = pks(prom>0.1);
                  locs = locs(prom>0.1);
              end

              if(isempty(pks)) % If there are no peaks, choose the last point
                  peak_idx = length(section_energy);
                  peak_val = section_energy(end);             
              else % if there are more than one peak; pick the second
                      peak_val = pks(1);
                      peak_idx = locs(1);
              end


             barrier.val(i,j)   = peak_val - minima.pitch.e(i);
             barrier.angle(i,j) = swept_angles(j);
             if(barrier.val(i,j)<0)
                 barrier.val(i,j) = 0;
             end
            
             

             % update the projection of barriers
             barrier.pitch(i,j)  = section_pitch(peak_idx);
             barrier.roll(i,j)   = section_roll(peak_idx);
             barrier.energy(i,j) = section_energy(peak_idx);
             
             
        end
       1;

       % update the plot elements
       [pel.b_edge.XData , pel.b_edge.YData] = pol2cart(barrier.angle(i,:)*pi/180 , barrier.val(i,:)); 
       pel.b_edge.XData = pel.b_edge.XData([2:end-1 2]);
       pel.b_edge.YData = pel.b_edge.YData([2:end-1 2]);
       pel.b_edge.XData = medfilt1(pel.b_edge.XData,5);
       pel.b_edge.YData = medfilt1(pel.b_edge.YData,5);
       pel.b_fill.XData = pel.b_edge.XData;
       pel.b_fill.YData = pel.b_edge.YData;
       pel.b_fill.FaceAlpha = 0.3;
       pel.b_fill.EdgeAlpha = 0.3;


     drawnow
i
end

close all

save ../data-animal/animal_barrier.mat barrier
