function posflucmpplot(pos0, pos1, t0, t1, odr)
% pos fluctuation comparison & plot.
%
% Prototype: posflucmpplot(pos0, pos1, t0, t1, odr)
% Inputs: pos0 - reference position array
%         pos1 - position to be compared
%         t0 - start time in second.
%         t1 - end time in second.
%         odr - ploy fit order.
%
% Example:
%   posflucmpplot(gps, avp, 100, 110);
%
% See also  avpcmpplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/10/2021
    if nargin<5, odr=2; end
    if nargin<4, t1=min(pos0(end,end),pos1(end,end)); end
    if nargin<3, t0=max(pos0(1,end),pos1(1,end)); end
    pos0 = datacut(pos0, t0, t1); pos1 = datacut(pos1, t0, t1);
    eth = earth(pos0(1,end-3:end-1)');
    t0 = pos0(:,end)-pos0(1,end); t1 = pos1(:,end)-pos1(1,end); 
    for k=3:-1:1
        pos0(:,end-k)=pos0(:,end-k)-pos0(1,end-k);  pos1(:,end-k)=pos1(:,end-k)-pos1(1,end-k);  % start from 0
        pp = polyfit(t0, pos0(:,end-k), odr);  % poly fit
        pos0(:,end-k) = pos0(:,end-k) - polyval(pp,t0);
        pp = polyfit(t1, pos1(:,end-k), odr);  % poly fit
        pos1(:,end-k) = pos1(:,end-k) - polyval(pp,t1);
    end
    myfig;
    plot(pos0(:,end), [pos0(:,end-3)*eth.RMh, pos0(:,end-2)*eth.clRNh, pos0(:,end-1)]); xygo('pos fluctuation'); 
    legend('Lat-ref', 'Lon-ref', 'Hgt-ref');
    plot(pos1(:,end), [pos1(:,end-3)*eth.RMh, pos1(:,end-2)*eth.clRNh, pos1(:,end-1)], '-.', 'linewidth',2);
    