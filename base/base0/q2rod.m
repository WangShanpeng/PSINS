function rod = q2rod(q, roddef)
% Convert transformation quaternion to Rodrigues parameters.
%
% Prototype: rod = q2rod(q, roddef)
% Inputs: q - transformation quaternion
%         roddef - Rod define-type, if define as rod=f(nrv)*u, than
%                  (1) roddef==1 for f(nrv)=tan(nrv/2),
%                  (2) roddef==2 for f(nrv)=tan(nrv/4),
%                  (3) roddef==3 for f(nrv)=2*tan(nrv/2)
%               where u=rv/nrv, nrv=norm(rv), rv for rotation vector.
% Output: rod - corresponding Rodrigues parameters
% 
% See also  rod2q, q2rv, m2rv.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 14/08/2019
    if nargin<2, roddef=1; end
    rv = q2rv(q);  nrv = sqrt(rv'*rv);
    if nrv>1e-20, rv = rv/nrv; end
    switch roddef
        case 1
            rod = tan(nrv/2)*rv;
        case 2
            rod = tan(nrv/4)*rv;
        case 3
            rod = 2*tan(nrv/2)*rv;
        otherwise
            error('roddef error!');
    end

