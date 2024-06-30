function mu = ff2mu(f, f1, uz)
% Trans two-specific force vectors (two IMU) to install angles.
%
% Prototype: mu = ff2mu(f, f1, uz)
% Inputs: f,f1 - two-specific force vectors
%         uz - installation angle of z-axis
% Output: mu - installation angles
%
% See also  aa2mu, aa2phi.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/02/2024
global glv
    if nargin<3, uz=0; end
    if size(f,2)==1; f=f'; f1=f1; end
    df = f1-f;
    ux = (uz*f(:,1)-df(:,2))./f(:,3);
    uy = (df(:,1)+uz*f(:,2))./f(:,3);
    if size(f,2)==1
        mu = [ux;uy;uz];
    else
        mu = [ux, uy];
        myfig, plot(mu/glv.deg); xygo('t', '\mu / \circ');
    end