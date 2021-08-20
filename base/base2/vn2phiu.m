function [phiu0, vnfit] = vn2phiu(vn, lti, isfig)
% Calculating yaw misalign angles from pure SINS velocity error.
%
% Prototype: [phiu0, vnfit] = vn2phiu(vn, lti)
% Inputs: vn - pure SINS velocity error, in most case for static base
%         lti - latitude
% Output: phiu0 - misalignment between calculating navigation frame and real
%               navigation frame
%         vnfit - polyfit for velocity
%
% See also  av2phiu, vn2phi.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/04/2021
global glv
    if nargin<3, isfig=0; end
    vnfit = vn;
    t0 = 2*vn(1,end)-vn(2,end); t = vn(:,end)-t0;
    pe = polyfit(t,vn(:,1),2); vnfit(:,1) = polyval(pe,t);  % 2 not 3 !
    pn = polyfit(t,vn(:,2),2); vnfit(:,2) = polyval(pn,t);
    pu = polyfit(t,vn(:,3),2); vnfit(:,3) = polyval(pu,t);
    phin0 = -pe(2)/glv.g0;
    phiu0 = phin0*tan(lti)-2*pn(1)/glv.g0/(glv.wie*cos(lti));
    if isfig==1
        myfig;
        subplot(131), plot(vn(:,end), [vn(:,1),vnfit(:,1)]); xygo('VE');
        subplot(132), plot(vn(:,end), [vn(:,2),vnfit(:,2)]); xygo('VN'); title(sprintf('\\phi_{U0} = %.4f(\\prime)', phiu0/glv.min));
        subplot(133), plot(vn(:,end), [vn(:,3),vnfit(:,3)]); xygo('VU');
    end
