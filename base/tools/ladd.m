function ladd(x, y, isabs)
% Selected line add.
%
% Prototype: ladd(x, y, isabs)
% Inputs: x, y - x, y to be addedd
%         isabs - is absolute x,y flag
% Output: N/A
% 
% See also lneg, lmul, laddy, lmc, lstyle, avplmc, tshift, scft0.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/10/2022
    if nargin<3, isabs=0; end
    if nargin<2, y=0; end
    if length(x)==2, y = x(2); x = x(1); end
    ob = findobj(gca);
    if isempty(ob), error('No line found.');  end
    for k=1:length(ob)
        p = get(ob(k));
        if strcmp(p.Selected,'on')==1
            x0 = get(ob(k),'XData');  y0 = get(ob(k),'YData');
            if isabs
                set(ob(k),'XData',x0-x0(1)+x);  set(ob(k),'YData',y0-y0(1)+y);
            else
                set(ob(k),'XData',x0+x);  set(ob(k),'YData',y0+y);
            end
            return;
        end
    end
    error('No line seclected.');