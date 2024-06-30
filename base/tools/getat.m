function [val,idx] = getat(array, tk, clm, method)
% Get value at tk from array.
%
% Prototype: val = getat(array, tk, method)
%
% See also  combinet, datacut, adddt.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/12/2019, 02/01/2024
    if length(tk)>1
        for k=1:length(tk)
            [val(k,:),idx(k,1)] = getat(array, tk(k));
        end
        return;
    end
    if tk<array(1,end) || tk>array(end,end),
        error(' tk out of range. ');
    else
        if nargin<3, clm=1:size(array,2)-1; end
        if nargin<4, method = 'nearest'; end
        val = interp1(array(:,end), array(:,clm), tk, method)';
    end
    if nargout==2, idx=find(array(:,end)>tk,1,'first'); end