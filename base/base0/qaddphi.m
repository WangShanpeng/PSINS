function qpb = qaddphi(qnb, phi)
% Get the calculated quaternion from accurate quaternion and misalignment
% angles. It can be denoted as 'qpb = qnb + phi', where qnb is accurate 
% quaternion and phi is misalignment angles.
%
% Prototype: qpb = qaddphi(qnb, phi)
% Inputs: qnb - attitude quaternion from ideal nav-frame to body-frame
%         phi - platform misalignment angles from computed ideal nav-frame to
%               nav-frame
% Output: qpb - attitude quaternion from computed nav-frame to body-frame
%
% See also  qdelphi, qq2phi, qaddafa, qdelafa, qq2afa, aaddmu.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 31/03/2008
    qpb = qmul(rv2q(-phi),qnb);
