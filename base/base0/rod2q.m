function q = rod2q(rod, roddef)
% Convert Rodrigues parameters to transformation quaternion.
%
% Prototype: q = rod2q(rod, roddef)
% Inputs: rod - Rodrigues parameters
%         roddef - Rod define-type, if define as rod=f(nrv)*u, than
%                  (1) roddef==1 for f(nrv)=tan(nrv/2),
%                  (2) roddef==2 for f(nrv)=tan(nrv/4),
%                  (3) roddef==3 for f(nrv)=2*tan(nrv/2)
%               where u=rv/nrv, nrv=norm(rv), rv for rotation vector.
% Output: q - corresponding transformation quaternion
% 
% See also  q2rod, rv2q, rv2m.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/08/2019
    if nargin<2, roddef=1; end
    rr = rod'*rod;
    switch roddef
        case 1
            q = [1; rod] / sqrt(1+rr);
        case 2
            q = [1-rr; 2*rod] / (1+rr);
        case 3
            q = [2; rod] / sqrt(4+rr);
        otherwise
            error('roddef error!');
    end

