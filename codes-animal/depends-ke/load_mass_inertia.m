% load mass-inertia properties for the animal
% enumerations
DL.HEAD = 1;
DL.MID  = 2;
DL.END  = 3;
DL.RMID = 4;
DL.REND = 5;
DL.LMID = 6;
DL.LEND = 7; 

% note:
% cofe = coxa + femur
% tita = tibia + tarsus

% mass -- all in gram
ani.front.m  = 0.85;
ani.back.m   = 1.85;
ani.cofe.l.m = 6.6e-2;
ani.tita.l.m = 1.3e-2;
ani.cofe.r.m = ani.cofe.l.m;
ani.tita.r.m = ani.tita.l.m;


    

% principal moments of inertia taken at the com -- g.mm^2 --> kg.m^2
ani.back.moi   = diag([52.29, 123.61, 163.70])/(1e9);
ani.front.moi  = diag([20.33,  22.27,  37.86])/(1e9);
% ani.cofe.l.moi = ani.cofe.l.m*(ani.cofe.l.d_av^.2)/12;
% ani.cofe.r.moi = ani.cofe.r.m*(ani.cofe.r.d_av^.2)/12;
% ani.tita.l.moi = ani.tita.l.m*(ani.tita.l.d_av^.2)/12;
% ani.tita.r.moi = ani.tita.r.m*(ani.tita.r.d_av^.2)/12;

% center of mass w.r.t body frame
ani.front.com_dx = [13.73-6.5 0 0]';
ani.back.com_dx  = [-6.45-6.5 0 0]';


