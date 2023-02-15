function igo = igoplot(igo, flag)
% SINS/GNSS/OD output plot for C++ class 'CSINSGNSSOD':
% void CSINSGNSSOD::operator<<(CFileRdWt &f)
% {
% 	f<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db  // 1-15
% 		<<sins.vnL<<sins.posL<<lvGNSS   // 16-24
% 		<<vnOD<<posOD<<lvOD<<ODKappa()  // 25-36
% 		<<dtGNSSdelay<<dyawGNSS<<(sins.Kg.e22-1.0) <<kftk; // 37-40
% }
%
% Prototype: igoplot(igo, flag)
% Inputs: igo - SINS/GNSS/OD data array.
%         flag - plot flag
%          
% See also  igplot, igoload, igkfplot, odpplot, imuplot, gpsplot, dataplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/01/2023
global glv
    if nargin<2, flag=0; end
    if ischar(igo), igo=adddt(binfile(igo,40),-flag); flag=0; end  % igoplot(igo_fname, t0);
	t = igo(:,end);
    if flag==0 || flag==1
        insplot(igo(:,[1:15,end]));
        subplot(323), plot(t, igo(:,16:18), '-.');  plot(t, igo(:,25:27), '--');
        dxyzGNSS = pos2dxyz(igo(:,[19:21,end]),igo(1,7:9)');  dxyzOD = pos2dxyz(igo(:,[28:30,end]),igo(1,7:9)');
        subplot(324), plot(t, dxyzGNSS(:,[2,1,3]), '-.'); plot(t, dxyzOD(:,[2,1,3]), '--');
    end
    if flag==0 || flag==2
        myfig
        subplot(321), plot(t, igo(:,22:24), '-.');  xygo('L');  plot(t, igo(:,31:33), '--');
        subplot(322), plot(t, igo(:,39)/glv.ppm);  xygo('dKGzz / ppm');
        subplot(323), plot(t, igo(:,[34,36])/glv.deg);  xygo('dPitch,dYaw / \circ'); 
        subplot(324), plot(t, igo(:,35));  xygo('Kod');
        subplot(325), plot(t, igo(:,37));  xygo('dT');
        subplot(326), plot(t, igo(:,38)/glv.deg); xygo('y');
    end

