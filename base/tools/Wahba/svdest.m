function [m, q, J] = svdest(B)
% SVD ESTimator (SVDEST)
% Example1:
%    [m, q, u, s, v] = svdest(randn(3));
% Example2:
%    glvs;
%    [imu, eth] = imustatic(avpchk([10*glv.deg; 30*glv.deg]));
%    B = -eth.gn*imu(1,4:6) + eth.wnie*imu(1,1:3) + cross(-eth.gn,eth.wnie)*cross(imu(1,4:6),imu(1,1:3));
%    Cnb = svdest(B);  att = m2att(Cnb)/glv.deg
% Example3:
%    rn1=[1;0;0]; rb1=[-1;0;0]; rn2=[0;1;0]; rb2=[0;1;0]; rn3=[0;0;1]; rb3=[0;0;1]; 
%    B = rn1*rb1'+rn2*rb2'+rn3*rb3';
%    Cnb = svdest(B)
%    norm(rn1-Cnb*rb1)+norm(rn2-Cnb*rb2)+norm(rn3-Cnb*rb3)
%
% See also  tr3, det3, inv3, adj3, svd3, foam, B33M44, quest, vortech, qrsvd

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/01/2020
    % assert(det(B)>0)
	[u, s, v] = svd(B);
%     m = u*v';  % make sure det(B)>0
%     m = u*diag([1;1;det(u*v')])*v';
    m = u*diag([1;1;sign(det(B))])*v';
    if nargout>=2, q = m2qua(m); end
    if nargout>=3
        s = diag(s); s(3) = sign(det(B))*s(3);
        J = sum(s);
    end
    