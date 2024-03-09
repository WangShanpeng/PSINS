function [q, r] = qrp(m)
% QR decomposition with positive diaganal R.
%
% Prototype: [q, r] = qrp(m)
%
% See also  qr.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 6/09/2023
    [q, r] = qr(m);
    for k=1:min(size(r))
        if r(k,k)<0, r(k,:)=-r(k,:); q(:,k)=-q(:,k); end
    end