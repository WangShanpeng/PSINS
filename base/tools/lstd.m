function [m, s] = lstd(t0, t1)
% Selected line std/mean.
%
% Prototype: [m, s] = lstd(t0, t1)
% Inputs: t0,t1 - start/end time (x-asix value)
% Outputs: m,s - mean/std
% 
% See also lsmooth, ladd, lmc.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/01/2023
    ob = findobj(gca);
    if isempty(ob), error('No line found.');  end
    for k=1:length(ob)
        p = get(ob(k));
        if strcmp(p.Selected,'on')==1
            t = get(ob(k),'XData');
            if nargin<2, t1=t(end); end
            if nargin<1, t0=t(1); end
            y = get(ob(k),'YData');
            idx0 = find(t0<=t,1,'first');
            idx1 = find(t1<=t,1,'first');
            m = mean(y(idx0:idx1)); s = std(y(idx0:idx1));
            title(sprintf('mean: %.3f;  std: %.3f', m, s));
            return;
        end
    end
    error('No line seclected.');