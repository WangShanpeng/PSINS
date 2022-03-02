function pvtplot(pos, ptype, xyz0)
% Plot single point position results.
%
% Prototype: pvtplot(pos, type, xyz0)
% Inputs: pos - recerver position/velocity etc.
%         ptype - plot type
%         xyz0 - reference/initial posxyz
%
% See also  obsplot, gpsplot.

% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/08/2015
global glv
    if nargin<2, ptype = ''; xyz0 = pos(1,1:3)'; end
    if nargin<3, xyz0 = pos(1,1:3)'; end
    ptype = lower(ptype);
    t = pos(:,end);
    [blh0, Cen] = xyz2blh(xyz0);
    xtext = 't / s';
    if ~isempty(strfind(ptype, 't0')), t = t-t(1)+mod(t(1)+blh0(2)/(2*pi)*86400,86400); xtext = 'local time t / s'; end
    if ~isempty(strfind(ptype, 'h')), t = t/3600; xtext = 'local time / h'; end
    myfigure;
    xyz = pos(:,1:3);  blh = xyz2blhBatch(xyz);
    xyz = [xyz(:,1)-xyz0(1), xyz(:,2)-xyz0(2), xyz(:,3)-xyz0(3)];
    blh = [blh(:,1)-blh0(1), blh(:,2)-blh0(2), blh(:,3)-blh0(3)];
    subplot(321), plot(t, xyz); xygo(xtext, '\DeltaXYZ / m');
    subplot(323), plot(t, [blh(:,1)*glv.Re,blh(:,2)*cos(blh(1,1))*glv.Re,blh(:,3)]); xygo(xtext, '\DeltaBLH / m');
    subplot(325), plot(t, pos(:,5)); xygo(xtext, 'residual error/m');
    subplot(322), plot(t, pos(:,4)/3e8); xygo(xtext, '\deltat / s');
    subplot(324), plot(t, pos(:,6)); xygo(xtext, 'satNum');
    subplot(326), plot(t, pos(:,7:11)); xygo(xtext, 'G/P/H/V/T/DOP');
    if sum(abs(pos(:,16)))>1e-3
        myfigure;
        subplot(221), plot(t, pos(:,12:14)); xygo(xtext, 'Vxyz / m/s');
        subplot(223), plot(t, pos(:,12:14)*Cen); xygo(xtext, 'Venu / m/s');
        subplot(222), plot(t, pos(:,15)/3e8); xygo(xtext, '\deltatDot / s/s');
        subplot(224), plot(t, pos(:,16)); xygo(xtext, 'residual error / m/s');
    end

