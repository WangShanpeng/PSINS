function lyawcvt(cvtstr)
% Selected line set to negtive value.
%
% Prototype: lyawcvt(cvtstr)
% Input: cvstr - convention descript string
%
% See also lneg, yawcvt, ladd, lmc, avplmc, tshift, scft0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/12/2022
    ob = findobj(gca);
    if isempty(ob), error('No line found.');  end
    for k=1:length(ob)
        p = get(ob(k));
        if strcmp(p.Selected,'on')==1
            y = get(ob(k),'YData');
            if nargin<1
                if isempty(find(y<0,1)), cvtstr='c360cc180'; else, cvtstr='cc180c360';  end
            end
            deg = pi/180;
            set(ob(k),'YData', yawcvt(y'*deg,cvtstr)/deg);
            return;
        end
    end
    error('No line seclected.');