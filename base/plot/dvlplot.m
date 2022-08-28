function dvlplot(dvl, sf)
% DVL or OD plot.
%
% Prototype: dvlplot(dvl)
% Inputs: dvl - [vb_right, vb_front, vb_up, t] or [vb_front, t]
%         sf - scale factor
%          
% See also  odplot, vnplot, imuplot, magplot, baroplot, insplot, inserrplot, kfplot, gpsplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/05/2015
    if nargin<2, sf = 1; end
    myfigure,
    if size(dvl,2)==2
        plot(dvl(:,2), dvl(:,1)*sf);
    else
        plot(dvl(:,4), [dvl(:,1:3),sqrt(dvl(:,1).^2+dvl(:,2).^2+dvl(:,3).^2)]*sf);
    end
	xygo('V');  title('DVL velocity');