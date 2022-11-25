function plotpsdn(data, ts)
% Plot PSD of multi-column data in each sub-figure.
%
% Prototype: plotpsdn(data, ts)
% Inputs: data - data to plot, 
%         ts - sample frequency
%
% Example:
%    plotpsdn(randn(100,13),0.01);
%
% See also  plotn.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/11/2022
    if nargin<2, ts=1; end
    for k=1:size(data,2), data(:,k)=abs(fft(data(:,k)-mean(data(:,k)))); end
    data = log10(data(2:fix(length(data)/2),:));
    frq = 1/ts/2/length(data)*(1:length(data));
    plotn(data, frq); xlabel('freq / Hz'); ylabel('log10');
