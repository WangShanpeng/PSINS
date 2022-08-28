function s = array2clbt(a, s, k)
% Translate column array to calibration struct.
%
% Prototype: s = array2clbt(a, s, k)
% Inputs: a - column array
%         s - input calibration struct
%         k - k-th calibration array element
% Output: s - calibration struct
%
% See also clbt2array, imutclbt, imuclbt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 10/08/2022
    if nargin>1 % k>0
        if k<=9, s.Kg(fix((k-1)/3)+1,rem(k-1,3)+1) = a;
        elseif k<=12, s.eb(k-9) = a;
        elseif k<=21, s.Ka(fix((k-13)/3)+1,rem(k-13,3)+1) = a;
        elseif k<=24, s.db(k-21) = a;
        elseif k<=27, s.Ka2(k-24) = a;
        elseif k<=30, s.rz(k-27) = a;
        elseif k<=33, s.rz(k-30) = a;
        elseif k<=36, s.rz(k-33) = a;
        else s.tGA = a;
        end
    else
        s.Kg = reshape(a(1:9),3,3)';  s.eb = a(10:12);
        s.Ka = reshape(a(13:21),3,3)';  s.db = a(22:24);  s.Ka2 = a(25:27);
        s.rx = a(28:30); s.ry = a(31:33); s.rz = a(34:36);
        s.tGA = a(37);
    end