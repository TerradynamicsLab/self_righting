% code to plot landscape trajectories

%% without landscape
clear 
close all
load  ../data-robot/robot_attempts.mat
load ../data-robot/landscape_full_wing_range.mat
load ../data-robot/barrier.mat

% flip roll of two attempts so that all trajectories go to same basin 
attempt{3}.eul_in{18}(:,1)=-attempt{3}.eul_in{18}(:,1);
attempt{1}.eul_in{40}(:,1)=-attempt{1}.eul_in{40}(:,1);    


frame_idx = [141 165 187];

barrier.XData(frame_idx(2),end) = barrier.XData(frame_idx(2),end-1);
barrier.YData(frame_idx(2),end) = barrier.YData(frame_idx(2),end-1);
barrier.XData(frame_idx(3),1) = barrier.XData(frame_idx(3),2);
barrier.YData(frame_idx(3),1) = barrier.YData(frame_idx(3),2);
barrier.XData(frame_idx(3),end) = barrier.XData(frame_idx(3),end-1);
barrier.YData(frame_idx(3),end) = barrier.YData(frame_idx(3),end-1);

% make barrier finer
for ii=1:3
    barrier.XDataFine{ii}=[];barrier.YDataFine{ii}=[];
for jj=2:numel(barrier.XData(1,:))
   barrier.XDataFine{ii} = [barrier.XDataFine{ii},linspace(barrier.XData(frame_idx(ii),jj-1), barrier.XData(frame_idx(ii),jj),10)];
   barrier.YDataFine{ii} = [barrier.YDataFine{ii},linspace(barrier.YData(frame_idx(ii),jj-1), barrier.YData(frame_idx(ii),jj),10)];
   
end
end

for ii = 1:3
    
    % make landscape interpolant 
    pel.itp{ii} = griddedInterpolant(surf_data{frame_idx(ii)}.XData',...
                                     surf_data{frame_idx(ii)}.YData',...
                                     surf_data{frame_idx(ii)}.ZData');
    
    
    figure(ii)
    clf
    hold on; box on
    colormap(firefly)
    
    ff.sh(ii) = surf(surf_data{frame_idx(ii)}.YData,...
                     surf_data{frame_idx(ii)}.XData,...
                     surf_data{frame_idx(ii)}.ZData,...
                     'edgealpha',0);   
    
   [cc ch] = contour3(surf_data{frame_idx(ii)}.YData,...
                     surf_data{frame_idx(ii)}.XData,...
                     surf_data{frame_idx(ii)}.ZData);
   ch.LevelList = linspace(ch.LevelList(1),ch.LevelList(end), 20);
   ch.LineColor= [1 1 1 ]*0.5
   
   plot3(barrier.YDataFine{ii},...
         barrier.XDataFine{ii},...
         pel.itp{ii}( barrier.XDataFine{ii},  barrier.YDataFine{ii}),...
         '--k', 'linewidth', 2);

   
   figure(5*ii)
   clf
    
   hold on; box on
   colormap(firefly)
    
   ff.sh(ii) = surf(surf_data{frame_idx(ii)}.YData,...
                     surf_data{frame_idx(ii)}.XData,...
                     surf_data{frame_idx(ii)}.ZData,...
                 'edgealpha',0); 
   [cc ch] = contour3(surf_data{frame_idx(ii)}.YData,...
                     surf_data{frame_idx(ii)}.XData,...
                     surf_data{frame_idx(ii)}.ZData);
    
   ch.LineColor = [1 1 1 ]*0
   ch.LevelList = linspace(ch.LevelList(1),ch.LevelList(end), 20);
   ch.LineColor = [1 1 1 ]*0.5
   
   plot3(barrier.YDataFine{ii},...
         barrier.XDataFine{ii},...
         pel.itp{ii}( barrier.XDataFine{ii},  barrier.YDataFine{ii}),...
         '--k', 'linewidth', 2);
   
    
    

    for jj = 1:attempt{ii}.num
        if(attempt{ii}.result(jj))
            cl = 'w'
            figure(ii)
            last_idx = max(find((unwrap(attempt{ii}.eul_fixed{jj}(:,1))<pi))) -1;
        else
            cl = 'k'
            figure(5*ii)
            last_idx = 101;
        end
        
        if(isempty(last_idx))
           last_idx=201; 
        end
        
     
         
         
      pp = plot(unwrap(attempt{ii}.eul_fixed{jj}(1:last_idx,1))*180/pi,...
            unwrap(attempt{ii}.eul_fixed{jj}(1:last_idx,2))*180/pi, ...
                       'color', cl)  ;   
       pp.ZData =    pel.itp{ii}(pp.YData, pp.XData)  +1          
%        title([num2str(ii) ', ' num2str(jj)]);

    end
    
    
%    
   
end


%% change figure prop
count = 1
for ii = [ 1 2 3 5 10 15]
    figure(ii)
    ylim([-0.5 1]*180);xlim([-1 1]*180);  
      set(get(gca,'yaxis'),'direction','reverse');
    set(get(gca,'xaxis'),'direction','normal');
    pbaspect([2 2 2])
    set(gca,'outerposition',[0 0 1 1 ]);
    axis normal
    set(gca,'plotboxaspectratio', [1.8 1.8 2])
    xticks([-2 -1 0 1 2]*90);  xticklabels({''});
    yticks([-2 -1 0 1 2]*90);  yticklabels({''});
    zticks([0 75 150]);  zticklabels({''});
%     zlim([0 150])
    set(gca,'fontsize',15, 'position',[0.02 0.02 0.95 0.95]);
    view(142 ,76);
    view(233 ,71);
    view(-125 ,75);
    view(-40 ,80);
    view(-116,48)
     view(65,30)
     view(42,76)
     set(gcf,'position',[20+(count-1)*500 50 360 400]);
%      caxis([40 420])
count=count+1
end
%%
 for i = [ 1 2 3 5 10 15]
    
     figure(i);
%      
%      set(gcf,'Renderer','Painter');
%      caxis ([50 500])
%      set(gca,'color','none')
%      print(gcf,['traj_on_pel_3d_' num2str(i) '.png'],'-dpng','-r600');  
  print(gcf,['traj_on_pel_3d_hires_cont_' num2str(i) '.png'],'-dpng','-r1200');  
%      print(gcf,['outputs\traj_on_pel_3d_' num2str(i) '.svg'],'-dsvg');
end


%%

figure(111)
clf;hold on
set(gca,'visible','off')
colormap(fire)
c1 = colorbar
