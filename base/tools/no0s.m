function varargout = no0s(varargin)
% To get non-zero data.
%
% Prototype: varargout = no0s(varargin)
%
% See also  no0, setvals.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/12/2021
    for k=1:nargout
        varargout{k} = no0(varargin{k});
    end
