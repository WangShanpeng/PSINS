% Sculling motion simulation.
% See also  demo_scull_error, demo_cone_motion, demo_cone_error.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/04/2012, 28/03/2014
clear all
glvs
A = 38*glv.deg*sin(0:1/10:2*pi);   len = length(A);  kk=1;
hfig = myfigure;
while 1
    if ~ishandle(hfig),  break;  end
    clf(hfig);
    kk = kk+1;  if kk>len; kk=1; end
    x = sin(A(kk)); y = 1-cos(A(kk));
    plot(x, y, 'o', 'linewidth', 10);
    annotation('arrow',[0.1 .5],[0.1,.5],'Color','r');
    title('Sculling Motion Simulation (by Yan G M)'); 
    xlim([-1, 1]); ylim([-1, 1]);
    pause(.2); 
end
