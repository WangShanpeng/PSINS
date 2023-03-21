function att = vn2att(vn, th, isfig)
% trans velocity to tracking attitude.
%
% Prototype: att = vn2att(vn, th, isfig)
% Inputs: vn - velocity in E/N/U direction
%         th - velocity threshold, if <th, then seen as 0.
%         isfig - figure flag
% Output: att - tracking attitude
%               [pitch,roll,yaw] = [atan(VU/|VEN|),0,atan(-VE/VN)]
%
% See also  pp2att, vn2dpos, vn2vbl, gps2avp, vn2phiu.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/12/2019
global glv
    if size(vn,1)==1 || size(vn,2)==1 
        vl = norm(vn(1:3));
        att = [atan2(vn(3), vl), 0, atan2(-vn(1), vn(2))]';
        return;
    end
    if nargin<3, isfig = 0; end
    if nargin<2, th = []; end
    if isempty(th), th = diff(vn(1:2,end)); end
    if size(vn,2)>7, vn = vn(:,[4:6,end]); end  % avp->v
    if size(vn,2)>4, vn = vn(:,[1:3,end]); end  % vp->v
    if size(vn,2)==1,  vn = vn'; vn(1,4)=0;   end
    vl = normv(vn(:,1:2));
    att = [atan2(vn(:,3), vl), vn(:,1)*0, atan2(-vn(:,1), vn(:,2)), vn(:,end)];
    if length(att)==4
        att = att(1:3)'; return;
    end
    idx = vl<th;
    att(idx,1:3) = 0;
    if isfig==1
        myfig;
        subplot(211), plot(vn(:,end), vn(:,1:3)); xygo('V');
        subplot(212), plot(att(:,end), att(:,1:3)/glv.deg); xygo('att');
    end
    if size(att,1)==1, att = att(1,1:3)'; end