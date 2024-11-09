function [x, y] = gcalim(isfix)
% Get current axis X/Y limits.
%
% Prototype: [x, y] = gcalim(isfix)
% Input: isfix - fix flag
% Outputs: x,y - X/Y limits
%
% See also  xlim, ylim.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/09/2024
    x = get(gca, 'xlim');
    y = get(gca, 'ylim');
    if nargin<1, isfix=1; end
    if isfix
        x = fix(x);
        y = fix(y);
    end
    if nargout<1
        fprintf('  %g', x);  fprintf('\n');
    end