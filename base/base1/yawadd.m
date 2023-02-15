function yaw = yawadd(yaw, dyaw, s)
% Yaw angles to add some dyaw.
%
% Prototype: yaw = yawadd(yaw, dyaw)
% Inputs: yaw - input yaw angles, in rad
%         dyaw - yaw to be added
%         s - sign for dyaw, +1 or -1
% Output: yaw - output yaw angles within [-pi, pi]
%
% Example
%    figure,  plot(yawadd((1:0.1:10)',1))
%
% See also  yawcvt, att2c.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/08/2022
    if nargin<3, s=1; end
    if size(yaw,2)>1  % if yaw is vector with time tag [ yaw, t ]
        if size(yaw,2)>3, y=3; else, y=1; end    % att = yawadd(att, dyaw)
        if length(dyaw)==1   % if dyaw is scale
            yaw(:,y) = yawadd(yaw(:,y), repmat(dyaw,size(yaw,1),1), s);
        elseif size(dyaw,2)>1
            dyaw = interp1(dyaw(:,end), dyaw(:,1), yaw(:,end), 'linear');
            yaw(:,y) = yawadd(yaw(:,y), dyaw, s);
            yaw(isnan(yaw(:,y)),:) = [];
        end
        return;
    end
    if s==1, yaw = yaw + dyaw;
    else     yaw = yaw - dyaw;    end
    yaw = atan2(sin(yaw),cos(yaw));