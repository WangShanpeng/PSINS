function vbl = vn2vbl(Cnb, vn)
% Convert vector expressed in n-frame to b-level-frame.
%
% Prototype: vbl = vn2vbl(Cnb, vn)
% Inputs: Cnb - SINS attitude DCM, (or Cnb=yaw)
%         vn - vector expressed in n-frame
% Output: vbl - vector expressed in b-level-frame
%
% See also  vn2vb, vn2att, vnplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/12/2014, 24/06/2021
    if size(Cnb,2)==1  % vbl = vn2vbl(yaw, vn)
        syaw = sin(Cnb); cyaw = cos(Cnb);
        if size(Cnb,1)==1, vn=vn'; end
        vbl = [cyaw.*vn(:,1)+syaw.*vn(:,2), -syaw.*vn(:,1)+cyaw.*vn(:,2), vn(:,3)];
        if size(Cnb,1)==1, vbl=vbl'; end
    elseif length(Cnb)==3
        spitch = Cnb(3,2);
        cpitch = sqrt(1-spitch*spitch);
        syaw = -Cnb(1,2)/cpitch;
        cyaw = Cnb(2,2)/cpitch;
        vbl = [cyaw*vn(1)+syaw*vn(2); -syaw*vn(1)+cyaw*vn(2); vn(3)];
    else % avblp = vn2vbl(avp)
        att = Cnb(:,1:3); vn = Cnb(:,4:6);
        s = sin(att(:,1:3)); c = cos(att(:,1:3));
        si = s(:,1); sj = s(:,2); sk = s(:,3); 
        ci = c(:,1); cj = c(:,2); ck = c(:,3);
        C = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
                cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
               -ci.*sj,             si,      ci.*cj           ];
        spitch = C(:,8);
        cpitch = sqrt(1-spitch.*spitch);
        syaw = -C(:,2)./cpitch;
        cyaw = C(:,5)./cpitch;
        vbl = Cnb;
        vbl(:,4:6) = [cyaw.*vn(:,1)+syaw.*vn(:,2), -syaw.*vn(:,1)+cyaw.*vn(:,2), vn(:,3)];
    end
