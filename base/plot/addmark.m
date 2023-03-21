function addmark(t, str)
% Add markers to all axes in current figure.
%
% Prototype: addmark(t, str)
% Inputs: t - time tag
%         str - line type string
%
% See also  adddt.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/03/2023
    if nargin<2, str='om'; end
    ax = findall(gcf, 'type', 'axes');
    for k=1:length(ax)
        subplot(ax(k)); hold on,
        plot(t, t*0, str, 'linewidth', 2);
    end
