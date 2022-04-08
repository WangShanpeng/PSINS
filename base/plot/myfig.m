function h = myfig(namestr)
% Short for myfigure.
%
% See also  myfigure.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2014, 23/03/2022
    if ~exist('namestr','var')
        h0 = myfigure;
    else
        if ~ischar(namestr)  % myfig(data);
            myfig; plot(namestr); grid on
            return;
        end
        h = myfigure(namestr);
    end
    if nargout==1
        h = h0;
    end