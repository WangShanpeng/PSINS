function diffplot(data)
% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/11/2020
    myfigure;
    data = diff(data,1,1);
    n = size(data,2);
    switch n
        case 1,
            plot(diff(data(:,1)));
        case 2,
            subplot(211), plot(data(:,1));
            subplot(212), plot(data(:,2));
        case 3,
            subplot(311), plot(data(:,1));
            subplot(312), plot(data(:,2));
            subplot(313), plot(data(:,3));
        case 4,
            subplot(221), plot(data(:,1));
            subplot(222), plot(data(:,2));
            subplot(223), plot(data(:,3));
            subplot(224), plot(data(:,4));
    end
