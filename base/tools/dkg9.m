function dkg = dkg9(dkgii, dkgij)
% Set gyro installation error vector, i.e. 
%  [ dkgxx, dkgxy, dkgxz;
%    dkgyx, dkgyy, dkgyz;
%    dkgzx, dkgzy, dkgzz ].
%
% Prototype: dkg = dkg9(dkgii, dkgij)
% Inputs: dkgii - scale factor error, in ppm
%         dkgij - cross installation angle error, in arcsec
% Output: dkg - installation error matrix, expressed as 9x1 vector
%
% See also  dkg1, dkg3, dka6.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
global glv
    if nargin<1, dkgii=0; end
    if nargin<2, dkgij=dkgii; end
    dkg = [ dkgii*glv.ppm, dkgij*glv.sec, dkgij*glv.sec;
            dkgij*glv.sec, dkgii*glv.ppm, dkgij*glv.sec;
            dkgij*glv.sec, dkgij*glv.sec, dkgii*glv.ppm ];
    dkg = dkg(1:9)';

