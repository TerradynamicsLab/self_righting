clear 

close all

[av_ke,~,alldat]= xlsread('..\data-animal\av_ke.xls')
%%
av_ke = reshape(av_ke,2,4)

rint_av = nanmean(av_ke(:,1));
rint_sd = nanstd(av_ke(:,1));

pint_av = nanmean(av_ke(:,2));
pint_sd = nanstd(av_ke(:,2));

rmod_av = nanmean(av_ke(:,3));
rmod_sd = nanstd(av_ke(:,3));

pmod_av = nanmean(av_ke(:,4));
pmod_sd = nanstd(av_ke(:,4));


%% barrier

load '..\data-animal\animal_barrier.mat'

% barrier
m = 2.7 %g
g = 9.81; 

wing_angle_list = 0:.5:90;
for i = 1:142%length(relative_com_height)
    valid_idx = find( barrier.angle(i,:)<=90 | barrier.angle(i,:)>=270);
    [min_barr,min_idx] = min(barrier.val(i,valid_idx));
    
    barrier.val_uj(i,:) = barrier.val(i,:)*m*g;
    barrier.roll_uj(i)  = min_barr*m*g;    
    barrier.pitch_uj(i) = interp1(barrier.angle(i,:), barrier.val_uj(i,:), 90);    
end


%%

figure(6)
clf
set(gcf,'position',[680 558 400 420])
set(gca,'fontsize',15,'position', [0.01 0.01 0.98 0.98]);
hold on; box on
[hb he] = barwitherr([pint_sd rint_sd;...
                      pmod_sd rmod_sd; ...
                            ]', ...
                     [pint_av rint_av;...
                      pmod_av rmod_av; ...
                            ]');  

set(hb(1), 'faceColor', 'w' , 'linewidth',1)
set(hb(2), 'faceColor', 'k', 'linewidth',1)

xlim([1 3]-0.5)
xticks([])
yticklabels('')

hb(1).FaceColor = 'flat'
hb(2).FaceColor = 'flat'
x1 = hb(1).XData;
x2 = hb(2).XData;

% hb.FaceColor = 'flat'
hb(1).CData = [1 1 1; 1 1 1];
hb(2).CData = ones(2,3)*0.7;
ylim([0 50])
yticks([0 20 40 60 80])
yticks([0 25 50])
set(gcf,'renderer','painters')
legend([hb(1) hb(2)],{'',''},'box','off')
%%

figure(3)
clf
set(gcf,'position',[980 229 400 400])
set(gca,'position',[0 0 1 1],'color','w')
hold on; box off; set(gca,'fontsize',15)
hold on; box on; set(gca,'fontsize',15)

pel.bmin.p = plot(wing_angle_list(1:142), barrier.pitch_uj,'b','linewidth',1.5);
pel.bmin.r = plot(wing_angle_list(1:142), barrier.roll_uj, 'r', 'linewidth', 1.5);

xlim([0, 1]*70);
% ylim([-1 1]*40);
ylim([0 700])
xticks([0 20 40 60])
yticks([0 200 400 600])

h1 = hline(rint_av, '--r'); set(h1,'linewidth',1.5);
hh = hline(pint_av, '--b'); set(hh,'linewidth',1.5);
h2 = hline(rmod_av, 'r'); set(h2,'linewidth',1.5);
hh = hline(pmod_av, 'b'); set(hh,'linewidth',1.5);


ax4 = axes('position',[0.4920 0.5350 0.4130 0.4325],'color','w')
hold on; box off; set(gca,'fontsize',15)
hold on; box on; set(gca,'fontsize',15)
pel.bmin.p = plot(wing_angle_list(1:142), smooth(barrier.pitch_uj),'b','linewidth',1.5);
pel.bmin.r = plot(wing_angle_list(1:142), smooth(barrier.roll_uj), 'r', 'linewidth', 1.5);


h1 = hline(rint_av, '--r'); set(h1,'linewidth',1.5);
hh = hline(pint_av, '--b'); set(hh,'linewidth',1.5);
h2 = hline(rmod_av, 'r'); set(h2,'linewidth',1.5);
hh = hline(pmod_av, 'b'); set(hh,'linewidth',1.5);

xlim([35 70]);ylim([0 60])
yticks([0 1 2 ]*30);yticklabels({''})
xticks([35 70]);
xticklabels({''})

