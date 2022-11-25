function val = vrbig(row,clm)
% Very big values.
%
% Prototype: val = vrbig(row,clm)
%
% See also  vrsmall.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/11/2022
    if nargin<1, row=1; end
    if nargin<2, clm=row; end
    val = ones(row,clm)*realmax/10; 