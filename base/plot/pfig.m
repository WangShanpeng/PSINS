function h = pfig(namestr, ylb)
% Short for myfigure.
%
% See also  myfigure, nextlinestyle.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/09/2024
    if ~exist('namestr','var')
        h0 = myfigure;
    else
        if ~ischar(namestr)  % myfig(data, ylabel);
            if nargin<2, ylb='val'; end
            pfig; plot(namestr); xygo(ylb);
            return;
        end
        h = myfigure(namestr);
    end
    if nargout==1
        h = h0;
    end