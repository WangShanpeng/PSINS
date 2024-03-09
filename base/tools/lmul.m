function lmul(val)
% Selected line set to multiply value.
%
% Prototype: lmul(val)
% 
% See also lneg, ladd, lmc, lsmooth, lmrs, tshift, scft0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/12/2022
    if nargin<1, val=10; end
    ob = findobj(gca);
    if isempty(ob), error('No line found.');  end
    for k=1:length(ob)
        p = get(ob(k));
        if strcmp(p.Selected,'on')==1
            y = get(ob(k),'YData');
            set(ob(k),'YData', val*y);
            return;
        end
    end
    error('No line seclected.');