function [rate, volume] = rimucfg(cfgUnitVectors, isfig)
% Redundancy IMU configuration rate.
%
% Prototype: [rate, volume] = rimucfg(cfgUnitVectors)
% Input: cfgUnitVectors - RIMU configuration unit vectors
% Outputs: rate - rate vectors
%          volume - volume from big to small
%
% Example
%   Cba = [ 1   0   0   -0.600   0.688;    0   1   0    0.405   0.647;   0   0   1   -0.689  -0.325 ];
%         % 0   1   0    0.405   0.647
%         % 0   0   1   -0.689  -0.325 ];
%   [rate, volume] = rimucfg(Cba', 1)
%
% See also  maxtetra.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/02/2023
    vs = [[0,0,0];cfgUnitVectors];
	[idx, v4, cbsV] = maxtetra(vs);
    res = cbsV(cbsV(:,1)==1,:);
    rate = res(:,2:4)-1;
    volume = res(:,end);
    if nargin<2 isfig==0; end
    if isfig==1
        myfig; nextlinestyle(-1);
        for k=2:length(vs)
            plot3([0,vs(k,1)],[0,vs(k,2)],[0,vs(k,3)],nextlinestyle(1),'linewidth',4); hold on
        end
        axis equal; grid on
    end