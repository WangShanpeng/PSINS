function varargout = tshift(varargin)
% Time tag shift to specific start time t0.
%
% Prototype: varargout = tshift(varargin)
% Examples: 1) [o1, o2, o3, dt] = tshift(i1, i2, i3, t0)
%           2) [o1, o2, o3, dt] = tshift(i1, i2, i3)   % t0=0
%
% See also  adddt, ladd, scft0.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2015
    t0 = varargin{1}(1,end); t00 = 0;
    kk = nargin;
    if length(varargin{kk})==1, t00=varargin{kk}(1); kk=kk-1; end
    for k=2:kk
        t0 = floor(min(t0, varargin{k}(1,end)));
    end
    varargout = varargin(1:kk);
    dt = t0-t00;
    for k=1:kk
        varargout{k}(:,end) = varargin{k}(:,end)-dt;
    end
    varargout{k+1} = dt;

