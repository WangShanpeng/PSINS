function [U, w] = ghpoint(n, odr)
% Gauss-Hermite quadrature points calculation.
% n - state dimension; odr = 3,5 
% U - Gauss points; w - weight
%
% See also  cubpoint, utpoint, vfbfx, vfbhx.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2023
    if nargin<2, odr=3; end
    if odr==3,     U0 = [1, -1];             w0 = [1 1]*sqrt(2*pi)/2;
    elseif odr==5, U0 = [1, 0 ,-1]*sqrt(3);  w0 = [1 4 1]*sqrt(2*pi)/6;  end
    U = U0; w = w0;
    for k=2:n
        U1 = repmat(U0, size(U,2), 1);   w1 = repmat(w0, size(U,2), 1);
        U = repmat(U, 1, length(U0));    w = repmat(w, 1, length(U0));
        U1 = reshape(U1, 1, size(U,2));  w1 = reshape(w1, 1, size(U,2));
        U = [U; U1];                     w = w.*w1;
    end
    w = w*1/(2*pi)^(n/2);

