function [U, w] = cubpoint(n, odr)
% Calculate 3th or 5th-order cubature points.
%
% Prototype: [U, w] = cubpoint(n, odr)
% Inputs: n - dimension
%         odr - = 3,5
% Outputs: U - cubature points
%          w - weights
%
% See also  utpoint, ghpoint, ckf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2022
    if nargin<2, odr=3; end
    if odr==3
        U = [eye(n), -eye(n)]*sqrt(n);  w = repmat(1/(2*n), 2*n, 1);
    elseif odr==5
        ei = eye(n); ein = -ei;
        pip = zeros(n,n*(n-1)/2); pin = pip;  kk = 1;
        for k=1:n
            for j=1:k-1
                pip(:,kk) = ei(:,j) + ein(:,k);
                pin(:,kk) = ei(:,j) - ein(:,k); kk = kk+1;
            end
        end
        U = [ei, ein, [pip,-pip,pin,-pin]/sqrt(2), zeros(n,1)]*sqrt(n+2);
        w = [repmat((4-n)/2/(n+2)^2,1,2*n), repmat(1/(n+2)^2,1,2*n*(n-1)), ...
             2/(n+2)];
    end
    