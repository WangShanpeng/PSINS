function att = atttrans(att, typ)
% Batch convertion for Euler attitude angles,
% yaw->pitch->roll to pitch->yaw->roll, or conversely.
%
% Prototype: att = atttrans(att, typ)
% Inputs: att - attitude in
%         typ - =1 for pitch->yaw->roll to yaw->pitch->roll
%               =0 for yaw->pitch->roll to pitch->yaw->roll
% Output: att - attitude out
%
% See also  q2att, q2att1, axxx2a.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2023
    if nargin<2, typ=1; end
	s = sin(att); c = cos(att);
	si = s(:,1); sj = s(:,2); sk = s(:,3); 
	ci = c(:,1); cj = c(:,2); ck = c(:,3);
    if typ==0, % yaw-pitch-roll to pitch-yaw-roll
        Cnb = [ cj.*ck-si.*sj.*sk, -ci.*sk,  sj.*ck+si.*cj.*sk, ...
                cj.*sk+si.*sj.*ck,  ci.*ck,  sj.*sk-si.*cj.*ck, ...
               -ci.*sj,             si,      ci.*cj           ];
        Cnb=Cnb(:,[5,6,4,8,9,7,2,3,1]);   % rfu->fur
        att(:,1:3) = [ atan2(Cnb(:,4),Cnb(:,1)), ...
                       atan2(Cnb(:,8),Cnb(:,9)), ...
                      -asin(Cnb(:,7)) ];
%     att = [ atan2(Cnb(2,1),Cnb(1,1));
%             atan2(Cnb(3,2),Cnb(3,3)); 
%             -asin(Cnb(3,1)) ];
    else
        Cnb = [ ci.*ck, -si.*cj+ci.*sj.*sk,  si.*sj+ci.*cj.*sk, ...
                si.*ck,  ci.*cj+si.*sj.*sk, -ci.*sj+si.*cj.*sk, ...
               -sk,      sj.*ck,             cj.*ck           ];
        Cnb=Cnb(:,[9,7,8,3,1,2,6,4,5]);   % fur->rfu
        att(:,1:3) = [ asin(Cnb(:,8)), ...
                       atan2(-Cnb(:,7),Cnb(:,9)), ... 
                       atan2(-Cnb(:,2),Cnb(:,5)) ];
%     att = [ asin(Cnb(3,2));
%             atan2(-Cnb(3,1),Cnb(3,3)); 
%             atan2(-Cnb(1,2),Cnb(2,2)) ];
    end

