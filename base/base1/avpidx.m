function avp = avpidx(data, idx, isdeg, isyawcvt)
% Extract AVPT from a data array.
%
% Prototype: avp = avpidx(data, idx, isdeg, isyawcvt)
% Inputs: data - data contains A/V/P/T
%         idx - column index of A/V/P/T
%         isdeg - Att/[lat,lon] is in degree
%         isyawcvt - if yaw is in clockwise 0-360degree convension
% Output: avp - =[att,vn,pos,t]
%
% See also: avpcvt, yawcvt, imuidx, gpsidx, insplot.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/12/2020
global glv
    avp = data(:,abs(idx));
    if nargin<4, isyawcvt=0; end
    if nargin<3, isdeg=0; end
    if length(isdeg)==1, isdeg=[isdeg, isdeg]; end
    if isdeg(1)==1, avp(:,1:3)=avp(:,1:3)*glv.deg; end % att in deg
    if isdeg(2)==1, avp(:,7:8)=avp(:,7:8)*glv.deg; end % [lat,lon] in deg
    if isyawcvt==1, avp(:,3) = yawcvt(avp(:,3),'c360cc180'); end
    for k=1:length(idx)
        if(idx(k))<0, avp(:,k)=-avp(:,k); end
    end
    