clear

close all 

addpath(genpath('depends-ke'))

% trial =  1 or 2
trial =2

% leg ball mass
ani.ball.m = 0.% g;

timeStep=1/2000;

DL.HEAD = 1;
DL.MID  = 2;
DL.END  = 3;
DL.RMID = 4;
DL.REND = 5;
DL.LMID = 6;
DL.LEND = 7; 

dlc.c1_2d = csvread(['../data-animal/leg_tracking/intact/int_t' num2str(trial) '_c1.csv']); 
dlc.c2_2d = csvread(['../data-animal/leg_tracking/intact/int_t' num2str(trial) '_c2.csv']); 


calib = csvread(['../data-animal/leg_tracking/intact/calib_t' num2str(trial) '.csv'])

n_frames = size(dlc.c1_2d,1);



for ii = 1:7
      dlc.p2d{ii} = [dlc.c1_2d(:,2*ii-1), dlc.c1_2d(:,2*ii), ...
                     dlc.c2_2d(:,2*ii-1), dlc.c2_2d(:,2*ii)];                           
    
    for jj=1:4
       dlc.p2d_pcut{ii}(:,jj)=fillmissing(medfilt1(dlc.p2d{ii}(:,jj),15),'linear','endvalues','nearest') 
    end
    
     dlc.p3d_pcut{ii} = dlt_reconstruct3D_RO(calib, dlc.p2d_pcut{ii});
     dlc.p3d_pcut{ii}(:,3)=dlc.p3d_pcut{ii}(:,3)+6.35+1.2; 
    
end 





for ii = 1:7
   for jj=1:3
       dlc.p3d_pcut{ii}(:,jj)=fillmissing(dlc.p3d_pcut{ii}(:,jj),'pchip','endvalues','nearest'); 
    end
end


load_animal_geometry

find_vel_ke_components


[nanmean(ke.r.all) nanmean(ke.p.all)]*1000
%%

figure(1)
clf
hold on

plot((ke.p.all)*1000,'b')
plot((ke.r.all)*1000,'r')
% xlim([0 2000])
yticklabels({''})
xticklabels({''})
xticks([0:2]*1000)
yticks([0:3]*50)
box on
ylim([0 150])

figure(3)
clf
hold on
plot3(ani.tip.l.prel(:,1),ani.tip.l.prel(:,2),ani.tip.l.prel(:,3),'b', 'linewidth',2)
plot3(ani.tip.r.prel(:,1),ani.tip.r.prel(:,2),ani.tip.r.prel(:,3),'r', 'linewidth',2)
axis equal
view(114,32)
box off
set(gca,'color','none')
set(gcf,'color','none')
set(get(gca,'xaxis'),'visible','off')
set(get(gca,'yaxis'),'visible','off')
set(get(gca,'zaxis'),'visible','off')
set(gcf,'renderer','painters')