function Cnb = a2matBatch(att)
% Convert Euler angles to direction cosine matrix(DCM) in batch processing.
%
% Prototype: Cnb = a2matBatch(att)
% Input: att - =[pitch, roll, yaw, t]
% Output: Cnb - =[C11, C12, C13, C21, C22, C23, C31, C32, C33, t]
%
% See also  a2mat, m2att, q2attBatch.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    s = sin(att(:,1:3)); c = cos(att(:,1:3));
    si = s(:,1); sj = s(:,2); sk = s(:,3); 
    ci = c(:,1); cj = c(:,2); ck = c(:,3);
    Cnb = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
            cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
           -ci.*sj,             si,      ci.*cj ];
    if size(att,2)>3, Cnb(:,10)=att(:,end); end   % time tag
    