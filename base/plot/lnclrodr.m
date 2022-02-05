function lnclrodr(odr)
% Line color order setting for current axis 'gca'.
%
% Prototype: lnclrodr(odr)
% Input: odr - line color order number, like 123, 145, 34, etc.
% 
% See also  NA.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/01/2022
    while 1
        if odr(1)<10, break; end
        odr = [floor(odr(1)/10), mod(odr(1),10), odr(2:end)];
    end
    odr = fliplr(odr);
    h = gca;
    clr = get(h,'ColorOrder');
    L = findobj(gca,'Type','Line');
    for k=1:min(length(L),length(odr))
        set(L(k), 'Color', clr(mod(odr(k),7),:));
    end