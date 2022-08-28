function varargout = interp1n(x1t1, varargin)
% Interpolation with different data length & the same time tag.
%
% Prototype: varargout = interp1n(x1t1, varargin)
% Inputs: x1t1 - [x1,t1]
%         varargin - [x2t2, x3t3, ..., xktk, t, method]
% Output: varargout - {x1t, x2t, ... xkt]
%
% See also interplim, smoothol.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2020
    method = 'linear';
    if ischar(varargin{end}), method = varargin{end}; varargin = varargin{1:end-1}; end
    t = varargin{end};
    varargin = {x1t1, varargin{1:end-1}};  % varargin = {x1t1, x2t2, x3t3, ..., xktk}
    tmin = zeros(length(varargin),1); tmax = tmin;
    for k=1:length(varargin)
        [~,idx] = sort(varargin{k}(:,end));
        varargin{k} = norep(varargin{k}(idx,:),size(varargin{k},2));
        tmin(k) = varargin{k}(1,end); tmax(k) = varargin{k}(end,end);
    end
    tmin = max(tmin); tmax = min(tmax);
    if length(t)==1  % if t is sampling interval
        t = (tmin:t:tmax)';
    else             % if t is sampling time points
        t = sort(t);
        t = t(find(t>=tmin,1,'first'):find(t<=tmax,1,'last'));
    end
    if isempty(t), error('time tag t error!'); end
    for k=1:length(varargin)
        varargout{k} = [interp1(varargin{k}(:,end), varargin{k}(:,1:end-1), t, method), t];
    end
