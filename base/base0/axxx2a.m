function att = axxx2a(axxx, axxstr)
% Convert any-direction Euler angles to flight yaw-pitch-roll Euler angle attitude.
%
% Prototype: att = axxx2a(axxx, axxstr)
% Inputs: axxx - attitude in xxx sequence
%         axxxstr - rotation direction string
% Output: att - yaw->roll->pitch (3-2-1) rotation sequence in R/x-F/y-U/z frame
% Test:
%    attr=[10;20;30]*glv.deg; [axxx2a(attr,'1x3z2y'), attrf(attr)]/glv.deg
%
% See also  a2mat, attrf, attrfu, atttrans.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/09/2021
    isclm = 0;
    if size(axxx,2)==1, axxx=axxx(1:3)'; isclm=1; end  % 3x1
    len = size(axxx,1);
    Cnb = repmat([1,0,0, 0,1,0, 0,0,1],len,1);
    for k=1:3
        att = zeros(len,3);
        att(:,upper(axxstr(2*k))-'X'+1) = axxx(:,axxstr(2*k-1)-'1'+1);
        s = sin(att(:,1:3)); c = cos(att(:,1:3));
        si = s(:,1); sj = s(:,2); sk = s(:,3); 
        ci = c(:,1); cj = c(:,2); ck = c(:,3);
        C = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
              cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
             -ci.*sj,             si,      ci.*cj ];
        Cnb = [ Cnb(:,1).*C(:,1) + Cnb(:,2).*C(:,4) + Cnb(:,3).*C(:,7), ...  % Cnb=Cnb*C
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
