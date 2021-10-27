function att = axxx2a(axxx, axxstr)
% Convert Euler angles to direction cosine matrix(DCM).
%
% Prototype: [Cnb, Cnbr] = a2mat(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Outputs: Cnb - DCM from navigation-frame(n) to body-frame(b), in yaw->pitch->roll
%                (3-1-2) rotation sequence
%          Cnbr - DCM in yaw->roll->pitch (3-2-1) roation sequence
% Test:
%   att0=randn(3,1)/10; [Cnb,Cnbr]=a2mat(att0); att=m2att(Cnb); [~,attr]=m2att(Cnbr); [att0, att, attr]
%
% See also  a2mat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/09/2021
    isclm = 0;
    if size(axxx,2)==1, axxx=axxx(1:3)'; isclm=1; end  % 3x1
    Cnb = repmat([1,0,0, 0,1,0, 0,0,1],size(axxx,1),1);
    for k=1:length(axxstr)
        if     axxstr(k)=='x'||axxstr(k)=='1', att = [axxx(:,k),  axxx(:,k)*0,axxx(:,k)*0];
        elseif axxstr(k)=='y'||axxstr(k)=='2', att = [axxx(:,k)*0,axxx(:,k),  axxx(:,k)*0];
        elseif axxstr(k)=='z'||axxstr(k)=='3', att = [axxx(:,k)*0,axxx(:,k)*0,axxx(:,k)  ]; end
        s = sin(att(:,1:3)); c = cos(att(:,1:3));
        si = s(:,1); sj = s(:,2); sk = s(:,3); 
        ci = c(:,1); cj = c(:,2); ck = c(:,3);
        C = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
              cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
             -ci.*sj,             si,      ci.*cj ];
        Cnb = [ Cnb(:,1).*C(:,1) + Cnb(:,2).*C(:,4) + Cnb(:,3).*C(:,7), ...
                Cnb(:,1).*C(:,2) + Cnb(:,2).*C(:,5) + Cnb(:,3).*C(:,8), ...
                Cnb(:,1).*C(:,3) + Cnb(:,2).*C(:,6) + Cnb(:,3).*C(:,9), ...
                Cnb(:,4).*C(:,1) + Cnb(:,5).*C(:,4) + Cnb(:,6).*C(:,7), ...
                Cnb(:,4).*C(:,2) + Cnb(:,5).*C(:,5) + Cnb(:,6).*C(:,8), ...
                Cnb(:,4).*C(:,3) + Cnb(:,5).*C(:,6) + Cnb(:,6).*C(:,9), ...
                Cnb(:,7).*C(:,1) + Cnb(:,8).*C(:,4) + Cnb(:,9).*C(:,7), ...
                Cnb(:,7).*C(:,2) + Cnb(:,8).*C(:,5) + Cnb(:,9).*C(:,8), ...
                Cnb(:,7).*C(:,3) + Cnb(:,8).*C(:,6) + Cnb(:,9).*C(:,9) ];
    end
    att = [ asin(Cnb(:,8)), ...
            atan2(-Cnb(:,7),Cnb(:,9)), ...
            atan2(-Cnb(:,2),Cnb(:,5)) ];
    if size(axxx,1)>size(axxstr,1), att(:,4)=axxx(:,end); end
    if isclm==1, att=att'; end
