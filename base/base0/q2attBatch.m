function att = q2attBatch(qnb)
% Convert nX4 attitude quaternion to nX3 Euler attitude angles.
%
% Prototype: att = q2attBatch(qnb)
% Input: qnb - attitude quaternion, nX4 array
% Output: att - Euler angles att=[pitch, roll, yaw] nX3 array in radians
%
% See also  q2att, a2quaBatch, a2matBatch, amulvBatch.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/11/2021

%     att = m2att(q2mat(qnb));

    q11 = qnb(:,1).*qnb(:,1); q12 = qnb(:,1).*qnb(:,2); q13 = qnb(:,1).*qnb(:,3); q14 = qnb(:,1).*qnb(:,4); 
    q22 = qnb(:,2).*qnb(:,2); q23 = qnb(:,2).*qnb(:,3); q24 = qnb(:,2).*qnb(:,4);     
    q33 = qnb(:,3).*qnb(:,3); q34 = qnb(:,3).*qnb(:,4);  
    q44 = qnb(:,4).*qnb(:,4);
    C12=2*(q23-q14);
    C22=q11-q22+q33-q44;
    C31=2*(q24-q13); C32=2*(q34+q12); C33=q11-q22-q33+q44;
    att = [ asin(C32), atan2(-C31,C33), atan2(-C12,C22) ];
    if size(qnb,2)>4, att(:,4) = qnb(:,end); end
    