function addyawplot(yaw, dyaw)
% Add yaw plot to insplot
%
% Prototype: addyawplot(yaw)
% Inputs: yaw - [yaw,t] array. 
%         dyaw - yaw bias
%
% See also  insplot, addtemplot.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/07/2021
global glv
    if nargin<2, dyaw=0; end
    if size(yaw,2)>4, yaw=yaw(:,[3,end]); end   % yaw=avp...
    subplot(322), plot(yaw(:,end), (yaw(:,end-1)+dyaw)/glv.deg, 'm');  legend('INS', 'GNSS');