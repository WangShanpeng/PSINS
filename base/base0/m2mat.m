function mo = m2mat(mi, dirstr)
% Convert attitude quaternion to Euler attitude angles.
%
% Prototype: mo = m2mat(mi, dirstr)
% Inputs: mi - input DCM
%         dir - direction string, include '+-XYZ' w.r.t. old 'XYZ' direction
% Output: mo - output DCM
%
% Example1, RFU->FUR
%   Cnb = [-1 0 0.008; 0.008 0 1; 0 1 0];  Cnb = mnormlz(Cnb);  Cnb1 = m2mat(Cnb, 'zxy');
%   [Cnb; Cnb1],  [m2att(Cnb), q2att1(m2qua(Cnb1))]/glv.deg
%
%   Cnb = a2mat([randn(2,1);0]);  Cnb1 = m2mat(Cnb, 'zxy');
%   [m2att(Cnb), q2att1(m2qua(Cnb1))]/glv.deg,  % if yaw=0, the same
%
% Example2, FUR->RFU
%   Cnb1 = a2mat(randn(3,1));  Cnb = m2mat(Cnb1, 'yzx');
%   [Cnb1; Cnb], [q2att1(m2qua(Cnb1)), m2att(Cnb)]/glv.deg
%
% See also  m2att, a2mat.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/08/2024
    if nargin<2, dirstr='+Y+Z+X'; end
    dirstr = upper(dirstr);
    if ~(dirstr(1)=='+'||dirstr(1)=='-'), dirstr=['+',dirstr]; end
    if ~(dirstr(3)=='+'||dirstr(3)=='-'), dirstr=[dirstr(1:2),'+',dirstr(3:end)]; end
    if ~(dirstr(5)=='+'||dirstr(5)=='-'), dirstr=[dirstr(1:4),'+',dirstr(5)]; end
    rvi = m2rv(mi);  rvo = rvi;
    for k=1:3
        stri = dirstr(2*k-1:2*k);
        switch stri
            case '+X',   rvo(1) =  rvi(k);
            case '-X',   rvo(1) = -rvi(k);
            case '+Y',   rvo(2) =  rvi(k);
            case '-Y',   rvo(2) = -rvi(k);
            case '+Z',   rvo(3) =  rvi(k);
            case '-Z',   rvo(3) = -rvi(k); 
%             case '+X',   rvo(k) =  rvi(1);
%             case '-X',   rvo(k) = -rvi(1);
%             case '+Y',   rvo(k) =  rvi(2);
%             case '-Y',   rvo(k) = -rvi(2);
%             case '+Z',   rvo(k) =  rvi(3);
%             case '-Z',   rvo(k) = -rvi(3); 
        end
    end
    mo = rv2m(rvo);
