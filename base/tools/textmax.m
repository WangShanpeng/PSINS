function textmax(x, y, tol)
% Put text on the curve.
%
% Prototype: textmax(x, y, tol)
% Inputs: x,y - x-y coordinate
%         tol - min value
%
% See also  plotline.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/11/2023
    if nargin<3, tol=0.05;  end
    if nargin==1, y=x; x=(1:length(y))'; end  % textmax(y) 
    if length(y)==1, tol=y; y=x; x=(1:length(y))'; end  % textmax(y, tol) 
    for k=1:size(y,2)
        [m,idx] = max(y(:,k));
        if m>tol, text(x(idx), y(idx,k), sprintf('%d',k)); end
    end