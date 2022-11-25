function imulvplot(lvx, lvy, lvz)
% SIMU inner lever parameter plot.
%
% Prototype: imulvplot(lv)
% Inputs: lvx, lvy, lvz - SIMU lever parameters
%
% See also  imulvest.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/11/2022
    if isstruct(lvx), lvx=[lvx.rx; lvx.ry; lvx.rz]; end  % imulvplot(clbt);
    if length(lvx)>6; lvz=lvx(7:9); lvy=lvx(4:6); lvx=lvx(1:3); elseif length(lvx)>3, lvy=lvx(4:6); lvx=lvx(1:3); end
    if ~exist('lvz','var'), lvz=zeros(3,1); end
    lvx=lvx-lvz; lvy=lvy-lvz; lvz=zeros(3,1);
    lvx=lvx*1000; lvy=lvy*1000; lvz=lvz*1000; 
    myfig
    plot(lvx(1),lvx(2),'>',lvy(1),lvy(2),'^',lvz(1),lvz(2),'*'); axis equal;
    xygo('x / mm', 'y / mm'); title('Inner level arm parameters');
    legend(sprintf('AXz: %4.1f',lvx(3)),sprintf('AYz: %4.1f',lvy(3)),sprintf('AZz: %4.1f',lvz(3)));
