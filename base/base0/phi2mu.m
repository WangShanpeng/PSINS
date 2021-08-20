function mu = phi2mu(phi, att0)
% Translate misalignment error angles(expressed in n-frame)
% to the installation error angles(expressed in b-frame).
%
% Prototype: mu = phi2mu(phi, att0)
% Inputs: phi - misalignment error angles between two IMUs
%         att0 - IMU Euler angles att0=[pitch0;roll0;yaw0]
% Output: mu - installation error angles
%
% See also  mu2phi, aa2phi, aa2mu.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/08/2020
	mu = -a2mat(att0)'*phi;
