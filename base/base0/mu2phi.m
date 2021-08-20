function phi = mu2phi(mu, att0)
% Translate the installation error angles(expressed in b-frame) 
% to misalignment error angles(expressed in n-frame).
%
% Prototype: phi = mu2phi(mu, att0)
% Inputs: mu - installation error angles between two IMUs
%         att0 - IMU Euler angles att0=[pitch0;roll0;yaw0]
% Output: phi - misalignment error angles
%
% See also  phi2mu, aa2phi, aa2mu.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/08/2020
	phi = -a2mat(att0)*mu;
