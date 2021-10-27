function avp = avpadderr(avp0, davp, israd, phimu)
% avp add some errors, it can be denoted as avp=avp0+davp, where
% avp0=[att0;vn0;pos0] and davp=[phi;dvn;dpos].
%
% Prototype: avp = avpadderr(avp0, davp, israd)
% Inputs: avp0 - avp0=[att0;vn0;pos0], avp initial values
%         davp - davp=[phi;dvn;dpos], avp errors
%         israd - phi/lat/lon unit is in rad
%         phimu - att err is phi or mu
% Output: avp - avp=[att; vn; pos], avp with errors
% 
% See also  avperrset, imuadderr, qaddphi, avpcmp, avpinterp, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/03/2014, 16/10/2020
global glv
    if nargin<4, phimu=1; end
    if nargin<3, israd=1; end
    if length(davp)<6, davp = [davp(1:3);zeros(6,1)]; % att err
    elseif length(davp)<9, davp = [davp(1:3);zeros(3,1);davp(4:6)]; end % att&pos err
    avp0 = avp0(:); davp = davp(:);
    if israd==0
        [phi, dvn, dpos] = setvals(davp(1:3).*[glv.sec;glv.sec;glv.min], davp(4:6), davp(7:9)./[glv.Re;glv.Re*cos(avp0(7));1]);
    else
        [phi, dvn, dpos] = setvals(davp(1:3), davp(4:6), davp(7:9));
    end
    if phimu==1
        avp(1:3,1) = q2att(qaddphi(a2qua(avp0(1:3)), phi));
    else
        avp(1:3,1) = aaddmu(avp0(1:3)', phi);
    end
    avp(4:9,1) = avp0(4:9)+[dvn; dpos];
