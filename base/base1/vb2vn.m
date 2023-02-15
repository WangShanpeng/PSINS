function vn = vb2vn(av, isavpflag)
% Convert vector expressed in b-frame to n-frame (batch process).
%
% Prototype: vn = vb2vn(av, isavpflag)
% Input: av - =[att,vb,t] or avp array
%        isavpflag - avp flag for output, vb = [att, vb, pos, t] 
% Output: vn - =[vE,vN,vU,t] or avp array
%
% See also  vy2vn, vn2vb, vn2vbl, vn2att, a2mat.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/07/2022
    if nargin<2, isavpflag=0; end
    if size(av,2)>9, isavpflag=1; end
    s = sin(av(:,1:3)); c = cos(av(:,1:3));
    si = s(:,1); sj = s(:,2); sk = s(:,3); 
    ci = c(:,1); cj = c(:,2); ck = c(:,3);
    Cnb = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
            cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
           -ci.*sj,             si,      ci.*cj           ];
    vn = [ Cnb(:,1).*av(:,4)+Cnb(:,2).*av(:,5)+Cnb(:,3).*av(:,6), ...    % vn = Cnb*vb;
           Cnb(:,4).*av(:,4)+Cnb(:,5).*av(:,5)+Cnb(:,6).*av(:,6), ...
           Cnb(:,7).*av(:,4)+Cnb(:,8).*av(:,5)+Cnb(:,9).*av(:,6), av(:,end) ];
    if isavpflag==1  % avnp = vn2vb(avbp, 1);
        vn = [av(:,1:3), vn(:,1:3), av(:,7:end)];
    end
    