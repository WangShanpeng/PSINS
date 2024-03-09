function plotn(data, t, isDelMean)
% Plot multi-column data in each sub-figure.
%
% Prototype: plotn(data, t, isDelMean)
% Inputs: data - data to plot, 
%         t - time flag
%
% Example:
%    plotn(randn(100,30));
%
% See also  dataplot, miniplot, plotpsdn, plotline.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/10/2022
    if nargin<3, isDelMean=0; end
    if isDelMean, for k=1:size(data,2), data(:,k)=data(:,k)-mean(data(:,k)); end, end
    if nargin<2, t=(1:size(data,1))'; end
    if length(t)==1, t=(1:size(data,1))'*t; end  % plotn(data, ts)
    if size(data,2)>9,
        plotn(data(:,1:9), t);
        data(:,1:9) = [];
        plotn(data, t);
        return;
    end
    myfig;
    switch size(data,2)
        case 1,
            plot(t, data(:,1));
        case 2,
            subplot(121), plot(t, data(:,1));            subplot(122), plot(t, data(:,2));
        case 3,
            subplot(311), plot(t, data(:,1));            subplot(312), plot(t, data(:,2));
            subplot(313), plot(t, data(:,3));
        case 4,
            subplot(221), plot(t, data(:,1));            subplot(222), plot(t, data(:,2));
            subplot(223), plot(t, data(:,3));            subplot(224), plot(t, data(:,4));
        case 5,
            subplot(321), plot(t, data(:,1));            subplot(322), plot(t, data(:,2));
            subplot(323), plot(t, data(:,3));            subplot(324), plot(t, data(:,4));
            subplot(325), plot(t, data(:,5));
        case 6,
            subplot(321), plot(t, data(:,1));            subplot(322), plot(t, data(:,2));
            subplot(323), plot(t, data(:,3));            subplot(324), plot(t, data(:,4));
            subplot(325), plot(t, data(:,5));            subplot(326), plot(t, data(:,6));
        case 7,
            subplot(331), plot(t, data(:,1)); subplot(332), plot(t, data(:,2)); subplot(333), plot(t, data(:,3));
            subplot(334), plot(t, data(:,4)); subplot(335), plot(t, data(:,5)); subplot(336), plot(t, data(:,6));
            subplot(337), plot(t, data(:,7));
        case 8,
            subplot(331), plot(t, data(:,1)); subplot(332), plot(t, data(:,2)); subplot(333), plot(t, data(:,3));
            subplot(334), plot(t, data(:,4)); subplot(335), plot(t, data(:,5)); subplot(336), plot(t, data(:,6));
            subplot(337), plot(t, data(:,7)); subplot(338), plot(t, data(:,8));
        case 9,
            subplot(331), plot(t, data(:,1)); subplot(332), plot(t, data(:,2)); subplot(333), plot(t, data(:,3));
            subplot(334), plot(t, data(:,4)); subplot(335), plot(t, data(:,5)); subplot(336), plot(t, data(:,6));
            subplot(337), plot(t, data(:,7)); subplot(338), plot(t, data(:,8)); subplot(339), plot(t, data(:,9));
    end
    