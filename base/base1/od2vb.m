function vb = od2vb(od, av, inst, kod, instav)
% Convert OD to b-frame velocity.
%
% Prototype: vb = od2vb(od, av, inst, kod, instav)
% Inputs: od - OD
%         av - reference attitude (PRY) & velocity (ENU) 
%         inst - ints=[dpitch;aos;dyaw], where dpitch and dyaw are
%            installation error angles(in rad) from odometer to SIMU
%            aos is Angle Of Slide coefficient
%         kod - odometer scale factor in meter/pulse.
%         instav - ints=[dpitch;0;dyaw] for av
% Output: vb - =[vR,vF,vU] in b-frame
%
% See also  vy2vn, vn2vb, vn2vbl.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/04/2023
global glv
    if nargin<5, instav=[0;0;0]; end
    if nargin<4, kod=1; end
    if nargin<3, inst=[0;0;0]; end
    ts = diff(av(1:2,end));
    aos = inst(2); inst(2)=0;  % AOS - angle of silde coefficient
    Cbo = a2mat(-inst)*kod;
	prj = Cbo*[0;1;0]; % from OD to SIMU
    od(:,1) = cumsum(od(:,1));
    [od, av] = interp1n(od, av, av(:,end));
    vel = [diff(od(:,1))./diff(od(:,end)),od(2:end,end)];
    wm = att2wm(av(:,[1:3,end]),0);
%     afa = -aos*vel(:,1).*wm(:,3)/ts;    s = sin(afa); c = cos(afa);
    afa = -aos*wm(:,3)/ts;    s = sin(afa); c = cos(afa);
    vb = [(prj(1)*c-prj(2)*s).*vel(:,1), (prj(1)*s+prj(2)*c).*vel(:,1), prj(3).*vel(:,1), vel(:,2)];
    av=av(2:end,:);
    av(:,1)=av(:,1)+instav(1);  av(:,3)=av(:,3)+instav(3);
    vb1 = vn2vb(av);
    %%
    myfig;  t=vb(:,end);
    subplot(221), plot(t, [vb(:,1), vb1(:,1)]);  xygo('V_R / (m/s)');
    subplot(223), plot(t, [vb(:,2), vb1(:,2)]);  xygo('V_F / (m/s)') 
    subplot(222), plotyy(t, av(:,3)/glv.deg, t, wm(:,3)/ts/glv.dps);  xygo('y') 
    subplot(224), plot(t, [vb(:,3), vb1(:,3), vb(:,2)-vb1(:,2)]);  xygo('V_U,dV_F / (m/s)'); 
    legend('V_{OD}', 'V_{Ref}', 'V_{F,OD}-V_{F,Ref}'); 
   
    