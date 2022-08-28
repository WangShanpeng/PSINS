function [att1, datt] = dov2att(dgn, att)
% Gravity abnomal & Deflection of vertical (DOV) display.
%
% Prototype: egmshow(data)
% Input1: dgn - Deflection of vertical (DOV)
%        att - attitude without DOV
% Outputs: att1 - attitude with DOV
%          datt - attitude errors between att & att1
%
% See also  gdovshow.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/08/2022
    if nargin<2
        att = zeros(3,1);
    end
    [imu, eth] = imustatic([att;zeros(6,1)],1,1);
    [~, att1] = dv2atti(eth.gn+dgn, eth.wnie, -imu(:,4:6)', imu(:,1:3)');
    datt = att1 - att;
