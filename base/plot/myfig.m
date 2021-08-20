function h = myfig(namestr)
% Short for myfigure.
%
% See also  myfigure.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2014
    if ~exist('namestr','var')
        h0 = myfigure;
    else
        h = myfigure(namestr);
    end
    if nargout==1
        h = h0;
    end