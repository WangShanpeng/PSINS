function ig = igplot(ig, flag)
% SINS/GNSS output plot for C++ class 'CSINSGNSS':
% void CSINSGNSS::operator<<(CFileRdWt &f)
% {
% 	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
% 		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
% 		<<dtGNSSdelay<<dyawGNSS <<kftk; // 25-27
% }
%
% Prototype: igplot(ig, flag)
% Inputs: ig - SINS/GNSS data array.
%         flag - plot flag
%          
% See also  igoplot, igkfplot, tfplot, igload, odpplot, imuplot, gpsplot, dataplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
global glv
    if nargin<2, flag=0; end
    if ischar(ig), ig=adddt(binfile(ig,27),-flag); flag=0; end  % igplot(ig_fname, t0);
	t = ig(:,end);
    if flag==0 || flag==1
        insplot(ig(:,[1:15,end]));
        subplot(323), plot(t, ig(:,16:18), '-.');
        dxyzGNSS = pos2dxyz(ig(:,[19:21,end]),ig(1,7:9)');
        subplot(324), plot(t, dxyzGNSS(:,[2,1,3]), '-.');
    end
    if flag==0 || flag==2
        myfig
        subplot(311), plot(t, ig(:,22:24), '-.');  xygo('L');
        subplot(312), plot(t, ig(:,25));  xygo('dT');
        subplot(313), plot(t, ig(:,26)/glv.deg); xygo('y');
    end

