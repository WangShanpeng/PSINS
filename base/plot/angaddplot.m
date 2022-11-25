function err = angaddplot(yaw, y180)
% Angle add plot, showing  the difference.
%
% Prototype: err = angaddplot(yaw, y180)
% Inputs: yaw - yaw data
%         y180 - 2nd yaw addition
% Output: err - error angle
%          
% See also  alignvntp.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/10/2022
global glv
    if nargin<2, y180=pi; end
    if size(yaw,2)>2, yaw = yaw(:,[3,end]); end
    len = 100;
    y0 = mean(yaw(1:len,1));
    y180 = y0 + y180; if y180>pi, y180=y180-2*pi; elseif y180<-pi, y180=y180+2*pi; end
    myfig;
    subplot(2,2,[1,3]);
    plot(yaw(:,end), yaw(:,1)/glv.deg,   [yaw(1,end);yaw(end,end)],[y0,y180; y0,y180]/glv.deg); xygo('y');
    subplot(2,2,2);
    plot(yaw(:,end), yaw(:,1)/glv.deg,   [yaw(1,end);yaw(end,end)],[y0,y180; y0,y180]/glv.deg); xygo('y');
    y = [yaw(1:len,1);y0];    ylim([min(y),max(y)]/glv.deg);
    subplot(2,2,4);
    plot(yaw(:,end), yaw(:,1)/glv.deg,   [yaw(1,end);yaw(end,end)],[y0,y180; y0,y180]/glv.deg); xygo('y');
    y = [yaw(end-len:end,1);y180];    ylim([min(y),max(y)]/glv.deg);
    err = mean(yaw(end-len:end,1)-y180);
    title(sprintf('Rotation error = %.3f (\\prime\\prime)', err/glv.sec));
    
    