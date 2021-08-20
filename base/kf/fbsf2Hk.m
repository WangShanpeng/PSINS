function Hk = fbsf2Hk(fbsf, ttype)
% Establish gyro g-sensitivity transition matrix, modeled by
%   ebxyz = kx*fx + ky*fy + kz*fz + kxx*fx^2 + kyy*fy^2 + kzz*fz^2 + kxy*fx*fy + kxz*fx*fz + kyz*fy*fz + eb0
%
% Prototype: Hk = fbsf2Hk(fbsf, ttype)
% Inputs: fbsf - specific force in b-frame;
%         ttype - transition matrix type, see the code
% Outputs: Hk - transition matrix Hk
%
% See also  kfhk.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/12/2020
    if nargin<2, ttype=1; end
    o13 = zeros(1,3); o16=zeros(1,6);
    fb = fbsf'; fb2 = fb.^2; fbX = [fb(1)*fb(2), fb(1)*fb(3), fb(2)*fb(3)];
    switch ttype
        case 1,
            Hk = [ [fb,o16,     1,0,0];
                   [o13,fb,o13, 0,1,0];
                   [o16,fb,     0,0,1] ];
        case 2,
            Hk = [ [fb,o16,     fb2,o16,     1,0,0];
                   [o13,fb,o13, o13,fb2,o13, 0,1,0];
                   [o16,fb,     o16,fb2,     0,0,1] ];
        case 3,
            Hk = [ [fb,o16,     fb2,o16,     fbX,o16,     1,0,0];
                   [o13,fb,o13, o13,fb2,o13, o13,fbX,o13, 0,1,0];
                   [o16,fb,     o16,fb2,     o16,fbX,     0,0,1] ];
        case 4,
            Hk = [ [fb,o16,     fbX,o16,     1,0,0];
                   [o13,fb,o13, o13,fbX,o13, 0,1,0];
                   [o16,fb,     o16,fbX,     0,0,1] ];
    end
%     Hk = Hk(:,1:end-3);
    