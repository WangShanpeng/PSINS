function dka = dka6(dkaii, dkaij)
% Set acc installation error vector, i.e. 
%  [ dkaxx, 0,     0;
%    dkayx, dkayy, 0;
%    dkazx, dkazy, dkazz ].
%
% Prototype: dka = dka6(dkaii, dkaij)
% Inputs: dkaii - scale factor error, in ppm
%         dkaij - cross installation angle error, in arcsec
% Output: dka - installation error matrix, expressed as 6x1 vector
%
% See also  dkg1, dkg3, dkg9.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
global glv
    if nargin<1, dkaii=0; end
    if nargin<2, dkaij=dkaii; end
    dka = [ dkaii*glv.ppm, 0,             0;
            dkaij*glv.sec, dkaii*glv.ppm, 0;
            dkaij*glv.sec, dkaij*glv.sec, dkaii*glv.ppm ];
    dka = dka([1:3,5:6,9])';

