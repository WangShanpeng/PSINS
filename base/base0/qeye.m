function qI = qeye(k)
% Generate unit quaternion.
% 
% Prototype: qI = qeye(k)
% Output: qI - unit quaternion i.e. qI(k)=1, and the other elements to be all 0. 
%
% See also  qconj, qmul.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/10/2019, 20/01/2021
    if nargin<1, k=1; end
    qI = [0;0;0;0];
    qI(k) = 1;
