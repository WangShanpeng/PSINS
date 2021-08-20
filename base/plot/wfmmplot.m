function wfm = wfmmplot(wfmmfile, ts, t0)
% Plot for IMU w/f MaxMin statistic.
%
% Prototype: wfmmplot(wfmmfile, ts, t0)
% Inputs: wfmmfile - wfmm file create by C++ 'CFileRdWt& CFileRdWt::operator<<(const CMaxMinn &mm)'
%         ts - sampling interval
%         t0 - stating time
%
% See also  xpplot, kfplot, inserrplot, kffile.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/01/2021
global glv
    if nargin<3, t0=0; end
    wfm = binfile(wfmmfile, 31);  wfm(:,end) = wfm(:,end) - t0;
    myfigure,
    subplot(321), plot(wfm(:,end), wfm(:,[1,2])/ts/glv.dps); xygo
    subplot(323), plot(wfm(:,end), wfm(:,[6,7])/ts/glv.dps); xygo
    subplot(325), plot(wfm(:,end), wfm(:,[11,12])/ts/glv.dps); xygo
    subplot(322), plot(wfm(:,end), diff(wfm(:,[17,16])')/ts/glv.g0); xygo
    subplot(324), plot(wfm(:,end), diff(wfm(:,[22,21])')/ts/glv.g0); xygo
    subplot(326), plot(wfm(:,end), diff(wfm(:,[27,26])')/ts/glv.g0); xygo
