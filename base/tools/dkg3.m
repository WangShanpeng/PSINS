function dkgz = dkg3(dkgii, dkgij)
% Set gyro installation error vector, i.e. [dkgxz;dkgyz;dkgzz].
%
% Prototype: dkgz = dkg3(dkgii, dkgij)
% Inputs: dkgii - scale factor error, in ppm
%         dkgij - cross installation angle error, in arcsec
% Output: dkgz - installation error vector
%
% See also  dkg1, dkg9, dka6.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
global glv
    if nargin<1, dkgii=0; end
    if nargin<2, dkgij=dkgii; end
    dkgz = [ 0,  0, dkgij*glv.sec;
             0,  0, dkgij*glv.sec;
             0,  0, dkgii*glv.ppm ];
    dkgz = dkgz(7:9)';

