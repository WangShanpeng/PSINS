function myplotyy(data, y1str, y2str, xstr)
% Plot two column data uning plotyy.
%
% Prototype: myplotyy(data, y1str, y2str, xstr)
% Inputs: data - = [y1,y2,t] array
%         y1str, y2str, xstr - label string.
%
% Example:
%    myfig; myplotyy(randn(100,2));
%
% See also  labeldef, myfig.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/06/2021
    if size(data,2)==2, data(:,3)=(1:length(data))'; end
    ax = plotyy(data(:,end),data(:,1), data(:,end),data(:,2)); grid on;
    if nargin<4, xlabel('\itt \rm/ s');
    else xlabel(xstr);  end
    if nargin>1, ylabel(ax(1), labeldef(y1str)); end
    if nargin>2, ylabel(ax(2), labeldef(y2str)); end
