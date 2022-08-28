function [pt, w] = cubpt5(n)
% Calculate 5th-order cubature points.
%
% Prototype: pt = cubpt5(n)
% Input: n - dimension
% Outputs: pt - cubature points
%          w - weights
%
% Example:
%   pt = cubpt5(3);  figure, sphere, hold on, plot3(pt(1,:), pt(2,:), pt(3,:), '*');
%
% See also  ckf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2022
    global cubpoint
    if isempty(cubpoint);
        for k=1:30, cubpoint{k}.pt = []; cubpoint{k}.w = []; end
    end
    if isempty(cubpoint{n}.pt)
        ei = eye(n); ein = -ei;
        pip = zeros(n,n*(n-1)/2); pin = pip;  kk = 1;
        for k=1:n
            for j=1:k-1
                pip(:,kk) = ei(:,j) + ein(:,k);
                pin(:,kk) = ei(:,j) - ein(:,k); kk = kk+1;
            end
        end
        cubpoint{n}.pt = [ei, ein, [pip,-pip,pin,-pin]/sqrt(2), zeros(n,1)];
        cubpoint{n}.w = [repmat((4-n)/2/(n+2)^2,1,2*n), repmat(1/(n+2)^2,1,2*n*(n-1)), 2/(n+2)];
    end
    pt = cubpoint{n}.pt;
    w  = cubpoint{n}.w;
    