function dkgzz = dkg1(dkgii)
% Set z-gyro scale factor error, i.e. dkgzz.
%
% Prototype: dkgz = dkg1(dkgii)
% Input: dkgii - scale factor error, in ppm
% Output: dkgzz - scale factor error, in 1.0e-6
%
% See also  dkg3, dkg9, dka6.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
global glv
    dkgzz = dkgii*glv.ppm;

