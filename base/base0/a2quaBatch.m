function qnb = a2quaBatch(att)
% Convert Euler angles to attitude quaternion.
%
% Prototype: qnb = a2quaBatch(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Output: qnb - attitude quaternion
%
% See also  a2qua, q2attBatch.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/05/2024
    
    att2 = att/2;
    s = sin(att2); c = cos(att2);
    sp = s(:,1); sr = s(:,2); sy = s(:,3); 
    cp = c(:,1); cr = c(:,2); cy = c(:,3); 
    qnb = [ cp.*cr.*cy - sp.*sr.*sy, ...
            sp.*cr.*cy - cp.*sr.*sy, ...
            cp.*sr.*cy + sp.*cr.*sy, ...
            cp.*cr.*sy + sp.*sr.*cy ];
%     idx = qnb(:,1)<0;   qnb(idx,:) = -qnb(idx,:);  % q0>=0