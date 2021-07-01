function [xp,yp,zp] = righting_energy_landscape(objectX,objectY,objectZ,objectCOM,rollStep,pitchStep)
% Function to calculate the potential energy landscape for a given object
% configuration

%  objectConfig - Meshgrid/PointCloud of the object

% RX = @(th)[ 1 0 0 ; 0 cosd(th) -sind(th) ; 0 sind(th) cosd(th) ];
% RY = @(th)[ cosd(th) 0 sind(th) ; 0 1 0 ; -sind(th) 0 cosd(th) ];
% RZ = @(th)[ cosd(th) -sind(th) 0 ; sind(th) cosd(th) 0 ; 0 0 1 ];

rollVal = [-180:rollStep:180]*pi/180;
pitchVal = [-180:pitchStep:180]*pi/180;
pel = zeros(3,length(rollVal)*length(pitchVal));

pointList = [objectX(1:end); objectY(1:end) ; objectZ(1:end)];
counter = 1;

[pitchMesh, rollMesh] = meshgrid([-180:rollStep:180]*pi/180, [-180:rollStep:180]*pi/180);

pitchMesh = pitchMesh(:);
rollMesh = rollMesh(:);

R = [-sin(pitchMesh)    cos(pitchMesh).*sin(rollMesh)  cos(pitchMesh).*cos(rollMesh)];

% we ignore this loop to do a matrix ultiplication right away; this is a
% tad confusing, but make it faster?
rotatedPointList = R*pointList;
rotatedCOM = R*objectCOM;

RelativeCOMHeight = rotatedCOM - min(rotatedPointList,[],2);

pel=[pitchMesh,rollMesh,RelativeCOMHeight];

xp = reshape(pel(:,1)*180/pi,sqrt(size(pel,1)),sqrt(size(pel,1)));
yp = reshape(pel(:,2)*180/pi,size(xp));
zp = reshape(pel(:,3),size(xp));

end





% function [xp,yp,zp] = PotEnergyLandscape_V4(objectX,objectY,objectZ,objectCOM,rollStep,pitchStep)
% % Function to calculate the potential energy landscape for a given object
% % configuration
% 
% %  objectConfig - Meshgrid/PointCloud of the object
% 
% % RX = @(th)[ 1 0 0 ; 0 cosd(th) -sind(th) ; 0 sind(th) cosd(th) ];
% % RY = @(th)[ cosd(th) 0 sind(th) ; 0 1 0 ; -sind(th) 0 cosd(th) ];
% % RZ = @(th)[ cosd(th) -sind(th) 0 ; sind(th) cosd(th) 0 ; 0 0 1 ];
% 
% rollVal = [-180:rollStep:180]*pi/180;
% pitchVal = [-180:pitchStep:180]*pi/180;
% pel = zeros(3,length(rollVal)*length(pitchVal));
% 
% pointList = [objectX(1:end); objectY(1:end) ; objectZ(1:end)];
% counter = 1;
% 
% [pitchMesh, rollMesh] = meshgrid([-180:rollStep:180]*pi/180, [-180:rollStep:180]*pi/180);
% 
% pitchMesh = pitchMesh(:);
% rollMesh = rollMesh(:);
% 
% R = [-sin(pitchMesh)    cos(pitchMesh).*sin(rollMesh)  cos(pitchMesh).*cos(rollMesh)];
% 
% 
% 
% 
% for  i = 1:length(pitchMesh)
%         
% % We just need the Z-coordinate to calculate the landscape, so do not need
% % to use the entire matrix to multiply
%         rotatedPointList = R(counter,:)*pointList;
%         rotatedCOM = R(counter,:)*objectCOM;
%         
%         RelativeCOMHeight = rotatedCOM - min(rotatedPointList);
%         pel(:,counter) = [pitchMesh(i);rollMesh(i);RelativeCOMHeight];
%         
% 
%         counter = counter+1;
% end
% 
%         xp = reshape(pel(1,:)*180/pi,sqrt(size(pel,2)),sqrt(size(pel,2)));
%         yp = reshape(pel(2,:)*180/pi,sqrt(size(pel,2)),sqrt(size(pel,2)));
%         zp = reshape(pel(3,:),sqrt(size(pel,2)),sqrt(size(pel,2)));
% end
% 
% 
% 
