function varargout = sortt(varargin)
% Sorting data by last column i.e. time tag.
%
% Prototype: varargout = sortt(varargin)
% Input: varargin - varargin{k}=[data,time_tag]
% Output: varargout - varargout{k}=[data,time_tag]_sorted
%
% See also  adddt, ttest.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2020
    method = 'ascend';
    if ischar(varargin{end}), method = varargin{end}; end
    for k=1:nargout
        [na, idx] = sort(varargin{k}(:,end), 1, method);
        varargout{k} = varargin{k}(idx,:);
    end