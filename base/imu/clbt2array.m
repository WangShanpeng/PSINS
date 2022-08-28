function a = clbt2array(s)
% Translate calibration struct to column array.
%
% Prototype: a = clbt2array(s)
% Input: s - calibration struct
% Output: a - column array
%
% See also array2clbt, imutclbt, imuclbt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 10/08/2022
    a = [reshape(s.Kg', 9,1); s.eb; reshape(s.Ka', 9,1); s.db; s.Ka2; s.rx; s.ry; s.rz; s.tGA; s.sf];   % according row 