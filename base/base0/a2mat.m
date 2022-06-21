function [Cnb, Cnbr] = a2mat(att)
% Convert Euler angles to direction cosine matrix(DCM).
%
% Prototype: [Cnb, Cnbr] = a2mat(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Outputs: Cnb - DCM from navigation-frame(n) to body-frame(b), in yaw->pitch->roll
%                (3-1-2) rotation sequence
%          Cnbr - DCM in yaw->roll->pitch (3-2-1) rotation sequence
% Test:
%   att0=randn(3,1)/10; [Cnb,Cnbr]=a2mat(att0); att=m2att(Cnb); [~,attr]=m2att(Cnbr); [att0, att, attr]
%
% See also  a2qua, m2att, m2qua, q2att, q2mat, attsyn, rv2m, rxyz, a2matBatch, axxx2a.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2008
    s = sin(att); c = cos(att);
    si = s(1); sj = s(2); sk = s(3); 
    ci = c(1); cj = c(2); ck = c(3);
    Cnb = [ cj*ck-si*sj*sk, -ci*sk,  sj*ck+si*cj*sk;
            cj*sk+si*sj*ck,  ci*ck,  sj*sk-si*cj*ck;
           -ci*sj,           si,     ci*cj           ];
    if nargout==2  % dual Euler angle DCM
        Cnbr = [ cj*ck, si*sj*ck-ci*sk, ci*sj*ck+si*sk;
                 cj*sk, si*sj*sk+ci*ck, ci*sj*sk-si*ck;
                -sj,    si*cj,          ci*cj            ];
    end
    