% Yaw-Pitch-Roll Euler angles demostration
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2024
glvs
pch=0; rll=0; yaw=0;
box3c([pch;rll;yaw]*glv.deg);
pause(1);
for yaw=0:10:70
    box3c([pch;rll;yaw]*glv.deg);    pause(0.2);
end
pause(1);
for pch=0:10:30
    box3c([pch;rll;yaw]*glv.deg);    pause(0.2);
end
pause(1);
for rll=0:10:50
    box3c([pch;rll;yaw]*glv.deg);    pause(0.2);
end