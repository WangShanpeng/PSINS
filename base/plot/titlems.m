function [m, s] = titlems(mnk, p)
% Add data mean&std title to the specific axis.
%
% Prototype: [m, s] = titlems(mnk, p)
% Inputs: mnk - subplot figure
%         p - precision specification
% Output: m,s - mean,std
%
% See also  avperrstd, meann, sumn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/06/2021
    if nargin<2, p=4; end
    if ~isempty(mnk), subplot(mnk); end
    ld = findall(gca,'type','line');
    str = sprintf('Mean=%%.%dg, Std=%%.%dg;  ', p, p);
    str1 = [];
    len = length(ld);
    for k=1:len
        y = get(ld(len-k+1,:),'Ydata');
        m(k,1) = mean(y);  s(k,1) = std(y);
        str1 = [str1, sprintf(str, m(k), s(k))];
    end
    title(str1);

