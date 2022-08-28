function val = getat(array, tk, clm, method)
% Get value at tk from array.
%
% Prototype: val = getat(array, tk, method)
%
% See also  combinet, datacut, adddt.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/12/2019
    if tk<array(1,end) || tk>array(end,end),
        error(' tk out of range. ');
    else
        if nargin<3, clm=1:size(array,2)-1; end
        if nargin<4, method = 'nearest'; end
        val = interp1(array(:,end), array(:,clm), tk, method)';
    end