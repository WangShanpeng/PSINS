function [dK, phi, mu] = Ka2dKphi(Ka, mu)
% IMU calibration matrix to scale factor, non-orthogonal rotation angles.
% Ref. '捷联惯导算法与组合导航原理' p98
%
% Prototype: [dK, phi, mu] = Ka2dKphi(Ka, mu)
% Input: Ka - calibration matrix or error matrix
%        mu - orthogonal rotation angles
% Outputs: dK - scale factor error
%          phi - non-orthogonal rotation angles
%          mu - orthogonal rotation angles
%
% See also  Ka2Cba.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 01/04/2024
    if abs(Ka(1,1))>0.9, dKa = eye(3)-Ka;  else, dKa=Ka; end
    dK = diag(dKa);
%     if nargin==2
%         phi=[dKa(3,2)+mu(1);dKa(3,1)-mu(2);dKa(2,1)+mu(3)];
%     else
%         phi = dKa+dKa';  phi=[phi(3,2);phi(3,1);phi(2,1)];
%         mu = [dKa(2,3);-dKa(1,3);dKa(1,2)];
%     end
    if nargin<2, mu=zeros(3,1); end
	phi = dKa+dKa';  phi=[phi(3,2);phi(3,1);phi(2,1)];
	mu = [dKa(2,3);-dKa(1,3);dKa(1,2)] - mu;
    
