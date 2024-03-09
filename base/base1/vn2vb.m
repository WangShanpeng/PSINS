function vb = vn2vb(av, isavpflag)
% Convert vector expressed in n-frame to b-frame (batch process).
%
% Prototype: vb = vn2vb(av, isavpflag)
% Input: av - =[att,vn,t] or avp array
%        isavpflag - avp flag for output, vb = [att, vb, pos, t] 
% Output: vb - =[vx,vy,vz,t] or avp array
%
% See also  vn2vbl, vb2vn, od2vb, vn2att, a2mat.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/06/2021
    if nargin<2, isavpflag=0; end
    if length(isavpflag)>1,  % vb = vn2vb(vn, att);
        att = isavpflag; isavpflag = 0;
        av = nonan([interp1(att(:,end), att(:,1:3), av(:,end), 'nearest'), av],1);
    end
    s = sin(av(:,1:3)); c = cos(av(:,1:3));
    si = s(:,1); sj = s(:,2); sk = s(:,3); 
    ci = c(:,1); cj = c(:,2); ck = c(:,3);
    Cnb = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
            cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
           -ci.*sj,             si,      ci.*cj           ];
    vb = [ Cnb(:,1).*av(:,4)+Cnb(:,4).*av(:,5)+Cnb(:,7).*av(:,6), ...    % vb = Cnb'*vn;
           Cnb(:,2).*av(:,4)+Cnb(:,5).*av(:,5)+Cnb(:,8).*av(:,6), ...
           Cnb(:,3).*av(:,4)+Cnb(:,6).*av(:,5)+Cnb(:,9).*av(:,6), av(:,end) ];
    if isavpflag==1  % avbp = vn2vb(avnp, 1);
        vb = [av(:,1:3), vb(:,1:3), av(:,[7:9,end])];
    end