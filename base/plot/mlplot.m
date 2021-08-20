function mlplot(data, lineType, lineColor)
% Multi-line plot function
%
% Prototype: mlplot(data, lineType, lineColor)
% Inputs: data - data to plot = [y, x], 
%         lineType,lineColor - line type & line color
% Example: 
%     mlplot(randn(100,1));
%
% See also  miniplot, msplot.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/02/2020
    if nargin<3, lineColor='m'; end
    if nargin<2, lineType='osdv^'; end
    if size(data,2)==1, data(:,2) = (1:length(data))'; end
    len = length(lineType);
    myfig;
    for k=1:len
        plot(data(k:len:end,2), data(k:len:end,1), [lineType(k),lineColor]); hold on;
    end
    grid on;