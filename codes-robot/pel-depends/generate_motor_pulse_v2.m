function [servoPulse ,vibPulse servoEndRiseTime]= generate_motor_pulse_v2(servoVer)
% generates the motor angles over time

% Load the motor response calculated earlier
load '../data-robot/raw_data/motor_pulse_v2.mat'
load '../data-robot/raw_data/vib_pulse_v2.mat'

% Find the starting time of the servo
[~, startIdx] = findpeaks(servoVer(:,3));
[~, vibStartIdx] = findpeaks(-servoVer(:,2)); %Need the negative symbol to detect the first peak -- otherwise detects the second peak


%Find the Actuation Angle
wingActuation = max(abs(servoVer(:,3)));
vibActuation =  max(abs(servoVer(:,2)));

if(vibActuation)
    startTime = servoVer(startIdx(1)-1,1);
    vibStartTime = servoVer(vibStartIdx(1)-1,1);
else
    startTime = servoVer(startIdx(1)-1,1);
    vibStartTime = startTime;
end



switch wingActuation
    case 60
        servoPulse = [[pulse.Start60LP(1,:)  pulse.Repeat60LP(1,:)]+startTime ;
                       pulse.Start60LP(2,:)  pulse.Repeat60LP(2,:) ; 
                       pulse.Start60RP(2,:)  pulse.Repeat60RP(2,:) ; 
                       pulse.Start60LR(2,:)  pulse.Repeat60LR(2,:) ; 
                       pulse.Start60RR(2,:)  pulse.Repeat60RR(2,:) ; 
                      ];        
    case 75
        servoPulse = [[pulse.Start75LP(1,:)  pulse.Repeat75LP(1,:)]+startTime ;
                       pulse.Start75LP(2,:)  pulse.Repeat75LP(2,:) ; 
                       pulse.Start75RP(2,:)  pulse.Repeat75RP(2,:) ; 
                       pulse.Start75LR(2,:)  pulse.Repeat75LR(2,:) ; 
                       pulse.Start75RR(2,:)  pulse.Repeat75RR(2,:) ; 
                      ];
     case 90
         servoPulse = [[pulse.Start90LP(1,:)  pulse.Repeat90LP(1,:)]+startTime ;
                        pulse.Start90LP(2,:)  pulse.Repeat90LP(2,:) ; 
                        pulse.Start90RP(2,:)  pulse.Repeat90RP(2,:) ; 
                        pulse.Start90LR(2,:)  pulse.Repeat90LR(2,:) ; 
                        pulse.Start90RR(2,:)  pulse.Repeat90RR(2,:) ; 
                      ];
    otherwise
         servoPulse = nan;
        
end

switch vibActuation
    case 0
        vibPulse = [tVib' + vibStartTime ,0.*Vib15];
    case 15
        vibPulse = [tVib' + vibStartTime,Vib15];
    case 30
        vibPulse = [tVib'+ vibStartTime,Vib30];
    case 45
        vibPulse = [tVib'+ vibStartTime,Vib45];
    otherwise
        vibPulse = [tVib'+ vibStartTime,Vib15]*nan;
end

servoPulse(2:5,1) =servoPulse(2:5,2);

servoPulse = servoPulse';


end