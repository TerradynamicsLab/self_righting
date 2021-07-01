% code to estimate the barriers about the pitch local minima
clear 
close all
 
load ../data-robot/landscape_full_wing_range.mat

wing_angle_list= -10:.5:90;
n_pts = length(wing_angle_list)-1;
wing_pct = 100*[0:n_pts]/(n_pts);

% load and organize the landscape data ; setup video writers
barrier = {};


%% setup the plots

figure(1);
clf;set(gcf,'position' , [8 55 506 558]);
hold on; box on; 
set(gca,'fontsize',15, 'position',[0.02 0.02 0.95 0.95]);
colormap(firefly);

% plot elements
pel.sh = surf(surf_data{1}.XData, surf_data{1}.YData ,surf_data{1}.ZData, 'edgealpha', 0);
pel.mh_pmin = plot(nan,nan,'marker','o','markersize', 8, 'markerfacecolor' , 'b','markeredgecolor','w');
pel.mh_rmin = plot(nan,nan,'marker','o','markersize', 8, 'markerfacecolor' , 'r','markeredgecolor','w');
pel.ph_barrier = plot(nan, nan, 'color',[0 0 0 0.6], 'linestyle','--','linewidth',2);

% set the plot view
xlim([-1 1]*180);ylim([-1 1]*180);  
set(get(gca,'xaxis'),'direction','reverse');
pbaspect([2 2 1.8])
set(gca,'outerposition',[0 0 1 1 ]);
axis normal
set(gca,'plotboxaspectratio', [1.8 1.8 2])
xticks([-2 -1 0 1 2]*90);  xticklabels({''});
yticks([-2 -1 0 1 2]*90);  yticklabels({''});
zticks([0 200 400]);  zticklabels({''});
set(gca,'fontsize',15, 'position',[0.02 0.02 0.95 0.95]);
view(142 ,76);
view(233 ,71);
view(-125 ,75);
view(-90,90);



figure(2)
clf;set(gcf,'position' , [734 370 240 240]);
hold on ; box on ; 
set(gca,'fontsize' , 15,'position',[0 0 1 1]);    
%plot elements
tt = 0:0.01:2*pi;
pel.b_edge = plot(nan,nan,'k','linewidth',2)
pel.b_fill = fill(0,0,'k')';
% pel.b_ke = plot(8.7*cos(tt),8.7*sin(tt), 'linewidth',1.5 , 'color',[1 0 0]);
% pel.b_ke = plot(15*cos(tt),15*sin(tt), 'linewidth',1.5 , 'color',[0 0 1]);
set(get(gca,'xaxis'),'visible','off')
set(get(gca,'yaxis'),'visible','off','direction','reverse')

axis equal
xlim([-1 1]*40);ylim([-1 1]*40)
hh = hline(0, 'k'); set(hh,'color',[0 0 0 0.3]);
hh = vline(0, 'k'); set(hh,'color',[0 0 0 0.3]);


    
    
    


%% code to calculate the barriers

% loop over all the points    

for i = 1:length(wing_angle_list)
      
       pel.sh.ZData = surf_data{i}.ZData;

       % make the surface interpolant for further calculation-- NOTE X=pitch, y=roll
       pel.itp = griddedInterpolant(surf_data{i}.XData',surf_data{i}.YData',surf_data{i}.ZData');
       
       % find pitch minima
       valid_idx = find(surf_data{i}.YData==0 & surf_data{i}.XData>-10 & surf_data{i}.XData<=90);
       valid_pitch = surf_data{i}.XData(valid_idx);
       valid_zcom  = surf_data{i}.ZData(valid_idx);
       [min_val, min_idx] = min(valid_zcom);
       minima.pitch.p(i) = valid_pitch(min_idx);
       minima.pitch.r(i) = 0;
       minima.pitch.e(i) = min_val;
       

       % add the minima markers
       pel.mh_pmin.XData = minima.pitch.p(i);
       pel.mh_pmin.YData = minima.pitch.r(i);
       pel.mh_pmin.ZData = minima.pitch.e(i)+5;       


      % Start ray tracing from average points         
      angle_step = 400;   % this is a temp variable used for finding boundaries on the the landscape
      angle_incr = sign(minima.pitch.r(i))*0.01;
      if(angle_incr ==0), angle_incr = 0.01;end
      end_points = [linspace( minima.pitch.r(i), -179.99,angle_step)',linspace(-180, -180,angle_step)' ;...                 
                    linspace(-180,-180,angle_step)',linspace( -180,179.99,angle_step)' ;...
                    linspace(-180, 179.99,angle_step)',linspace(180,180,angle_step)' ;...
                    linspace( 180, 180,angle_step)',linspace( 180, -179.99,angle_step)' ;...
                    linspace( 180, minima.pitch.r(i)-angle_incr,angle_step)',linspace(-180, -180,angle_step)' ];

     end_points(:,3) = wrapTo360(  atan2(  end_points(:,2)-minima.pitch.p(i), ...
                                           end_points(:,1)-minima.pitch.r(i)  ) ...
                                  *180/pi);
     end_points = sortrows(end_points,3);

     % Remove duplicate points
     [aa bb cc] = unique(end_points(:,3));
     end_points = end_points(bb,:);
     end_points = [180, minima.pitch.r(i),0 ; end_points ; 180, minima.pitch.r(i),360];

     % Get section of the landscape from average point to the end
     barrier_points = [];

     swept_angles = wrapTo360(linspace(0,360,500)); % how many points in the angular sweep
     sec_steps = 100; % how many data points in a section

     % radial sweep
        for j = 1:length(swept_angles)

              section_roll = linspace(minima.pitch.r(i),interp1(end_points(:,3),end_points(:,1),swept_angles(j)),sec_steps);
              section_pitch = linspace(minima.pitch.p(i),interp1(end_points(:,3),end_points(:,2),swept_angles(j)),sec_steps);
              section_energy = pel.itp(section_pitch ,section_roll);


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
%                   end
              end


             barrier.val(i,j) = peak_val - minima.pitch.e(i);
             barrier.angle(i,j) = swept_angles(j);
             if(barrier.val(i,j)<0)
                 barrier.val(i,j) = 0;
             end


             % update the projection of barriers
             pel.ph_barrier.XData(j) = section_pitch(peak_idx);
             pel.ph_barrier.YData(j) = section_roll(peak_idx);
             pel.ph_barrier.ZData(j) = section_energy(peak_idx);
             barrier.XData(i,j) = section_pitch(peak_idx);
             barrier.YData(i,j) = section_roll(peak_idx);
             barrier.ZData(i,j) = section_energy(peak_idx);
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
end        
 

save ../data-robot/barrier.mat barrier minima

    



