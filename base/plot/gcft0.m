function t0 = gcft0(t)
% Get the minimum xdata 't0' in gcf;
%
% Prototype: t0 = gcft0(t)
% Input: NA.
% Output: t0 - t0
%
% See also  scft0, xlimall.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/11/2022
global glv
global gcf_t00
    if nargin==1,
        if isempty(gcf_t00), gcft0(); end
        if t==0, t0=gcf_t00; end  % t0 = gcft0(0)
        return;
    end
    ax = findall(gcf, 'type', 'axes');
    for k=1:length(ax)
        if isempty(get(ax(k),'Tag'))
            t0 = inf;
            lin = findall(ax(k), 'type', 'line');
            for kk=1:length(lin)
                t = get(lin(kk), 'xdata');  t0 = min(t0,t(1));
            end
            t0 = fix(t0);
            gcf_t00 = t0; glv.t0 = t0;
            break;
        end
    end

