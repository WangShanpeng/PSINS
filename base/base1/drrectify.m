function [avpdr, inst, kod] = drrectify(avpdr, marker)
% Dead Reckoning(DR) trajectory rectified by land marker.
%
% Prototype: avp = drrectify(avpdr, marker)
% Inputs: avpdr - DR avp
%         marker - land marker = [lat,lon,hgt,t]
% Outputs: avpdr - avp after land marker rectified.
%          inst,kod - see drcalibrate.
%
% Example:
%    Cxyz = [0,0,0,1; 1000,100,10,100];  Mxyz=Cxyz(2,:);  Mxyz(1:3)=Mxyz(1:3)+10;
%    Cpos = dxyz2pos(Cxyz,[0;0;0]);  Mpos=dxyz2pos(Mxyz,[0;0;0]);
%    [Rpos, inst, kod] = drrectify(Cpos, Mpos');
%    pos2dplot(Cpos, Rpos); hold on, plot(Mxyz(1),Mxyz(2),'vr'); axis equal;
%    legend('xy0', 'Calcu', 'Rectify', 'Marker');
%    myfig; plot(Cxyz(1,4), 0, 'p', Cxyz(:,4),Cxyz(:,3),'b-', Rpos(:,4),Rpos(:,3),'r', Mxyz(4),Mxyz(3),'vr');
%    legend('h0', 'Calcu', 'Rectify', 'Marker'); xygo('hgt');
%   
% See also  drcalibrate, apmove, pos2dxyz, dxyz2pos.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 10/09/2021
    pos1Real = marker(1:3);
    pos0 = avpdr(1,end-3:end-1)';
	avp1 = getat(avpdr, marker(end)); pos1DR = avp1(end-2:end);
    [inst, kod] = drcalibrate(pos0, pos1Real, pos1DR);
    dxyz = pos2dxyz(avpdr(:,end-3:end));
    dxyz(:,1:3) = dxyz(:,1:3)*kod;
    dxyz(:,1:3) = dxyz(:,1:3)*a2mat([0;0;inst(3)]);  % leveling rectify
    dxyz(:,3) = dxyz(:,3) - normv(dxyz(:,1:2))*inst(1);  % vertical rectify
    avpdr(:,end-3:end) = dxyz2pos(dxyz, avpdr(1,end-3:end-1)');
    
