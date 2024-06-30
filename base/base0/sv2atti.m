function [qnb, att, Cnb] = sv2atti(vn, vb, yaw0)
% Using single-measurement vector to determine attitude.
%
% Prototype: [qnb, att, Cnb] = sv2atti(vn, vb, yaw0)
% Inputs: vn - reference vector in nav-frame
%         vb - measurement vector in body-frame
%         yaw0 - initial yaw setting
% Outputs: qnb, att, Cnb - the same attitude representations in 
%               quaternion, Euler angles & DCM form
% Example:
%    [qnb, att, Cnb] = sv2atti(gn, -fb);
%       where gn is gravity vector and fb is acc specific force vector.
%
% See also  dv2atti, mv2atti, fb2atti, rv2q.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/09/2012
    if nargin==1, yaw0=0; vb=vn; vn=[0;0;1];   % sv2atti(vb)
        if length(vb)>10, vb=mean(vb(:,end-3:end-1))'; end % sv2atti(imu)
    elseif nargin==2&&length(vb)==1, yaw0=vb; vb=vn; vn=[0;0;1];   % sv2atti(vb, yaw0)
        if length(vb)>10, vb=mean(vb(:,end-3:end-1))'; end % sv2atti(imu, yaw0)
    elseif nargin==2&&length(vb)==3, yaw0=0; end  % sv2atti(vn, vb)
    afa = acos(vn'*vb/norm(vn)/norm(vb));
    phi = cross(vb,vn);
    nphi = norm(phi);
%    if cos(afa/2)==0 ...
    if nphi<10e-20      % q = rv2q(phi/nphi*afa);
        qnb = [cos(afa/2); sin(afa/2)*[1;1;1]];
    else
        qnb = [cos(afa/2); sin(afa/2)*phi/nphi];
    end
    att = q2att(qnb);   att(3) = yaw0;
    [qnb, att, Cnb] = attsyn(att);

