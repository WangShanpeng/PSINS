% Yaw-Pitch-Roll Euler angles demostration
% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 25/01/2024
glvs
pch=0; rll=0; yaw=0;
% myfig
subplot(4,3,1); text(0,0.5, '◊™∂Ø≤ªø…ΩªªªŒÛ≤Ó—› æ(PSINS)');  axis off;
subplot(4,3,[4,7]);  
box3c([pch;rll;yaw]*glv.deg);  axis off;
pause(1);
%%
subplot(2,3,2);    title('»∆X÷·90°„');
q1 = [1;0;0;0]; q2 = q1;
for pch=0:10:90
    q1 = a2qua([pch;0;0]*glv.deg);
    box3c(qmul(q1,q2));  axis off;  pause(0.2);
end
pause(1);
subplot(2,3,3);    title('»∆Y÷·90°„');
for rll=0:10:90
    q2 = a2qua([0;rll;0]*glv.deg);
    box3c(qmul(q1,q2));  axis off;  pause(0.2);
end
pause(1);
%%
subplot(2,3,5);    title('»∆Y÷·90°„');
q1 = [1;0;0;0]; q2 = q1;
for rll=0:10:90
    q2 = a2qua([0;rll;0]*glv.deg);
    box3c(qmul(q2,q1));  axis off;  pause(0.2);
end
pause(1);
subplot(2,3,6);    title('»∆X÷·90°„');
for pch=0:10:90
    q1 = a2qua([pch;0;0]*glv.deg);
    box3c(qmul(q2,q1));  axis off;  pause(0.2);
end

