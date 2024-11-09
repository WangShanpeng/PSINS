function mat = maddv(mat, vel)
% Matrix add vector.
%
% Prototype: mat = maddv(mat, vel)
% Inputs: mat - matrix m*n
%         vel - m*1, n*1 or 1*m, 1*n vector
% Output: mat - matirx output
%
% Examples
%   maddv(randn(4,3), zeros(3,1));
%   maddv(randn(4,3), zeros(4,1));
%
% See also  delbias.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/09/2024
    [m, n] = size(mat);
    vel = vel(:);  col = length(vel);
    if m==col
        mat = mat + repmat(vel, 1, n);
    elseif n==col
        mat = mat + repmat(vel', m, 1);
    elseif n-1==col   % if the last column is time tag
        mat(:,1:end-1) = mat(:,1:end-1) + repmat(vel', m, 1);
    else
        error('Dimension mismatched! ');
    end