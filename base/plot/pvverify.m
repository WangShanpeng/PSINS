function pvverify(pos, vn)
% Pos(lat,lon,hgt) & vel (VE,VN,VU) consistence verify.
%
% Prototype: pvverify(pos, vn)
% Inputs: pos, vn - position & velocity array
%          
% See also  gpsplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/03/2023
    if size(pos,2)>7, pos=pos(:,4:end); end  % pvverify(avp);
    if nargin<2, vn=pos(:,[1:3,end]); pos=pos(:,4:end); end; % pvverify(vp);
     vn1 = pp2vn(pos);
     myfig;
     plot(vn(:,end), vn(:,1:3)), hold on, plot(vn1(:,end),vn1(:,1:3),'-.');  xygo('V');
     legend('v_E', 'v_N', 'v_U', 'v_E by pos', 'v_N by pos', 'v_U by pos');