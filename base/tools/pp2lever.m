function lever = pp2lever(ap, pos, isfig)
% Lever arm calculation between two position array, (one contains attitude info).
%
% Prototype: lever = pp2lever(ap, pos, isfig)
% Inputs: ap - [att, pos] array
%         pos - position array
%         isfig - figure flag
% Output: lever - lever parameter = [lv_x, lv_y, lv_z]^b, from ap_pos to pos
%
% Example:
%    lever = pp2lever(avp, gps, 1);
%
% See also  avplever, inslever, pp2vn.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/10/2021
    if size(ap,2)>7, ap=ap(:,[1:3,7:10]); end
    if size(pos,2)>7, pos=pos(:,7:10);
    elseif size(pos,2)>4, pos=pos(:,4:7);  end
    if nargin<3, isfig=0; end
    eth = earth(pos(1,1:3)');
    aperr = avpcmp(ap, [pos(:,1:3)*0, pos], 'noatt');
    aperr(:,4:6) = -[aperr(:,5)*eth.clRNh, aperr(:,4)*eth.RMh, aperr(:,6)];  % lv^n = pos - ap_pos
    lv = aperr(:,4:7);
    for k=1:length(aperr)
        lv(k,1:3) = aperr(k,4:6)*a2mat(aperr(k,1:3)');  % lv^n = Cnb * lv^b => lv^b' = lv^n' * Cnb
    end
    lever = mean(lv(:,1:3))';
    if isfig==1
        myfig;
        subplot(211), plot(aperr(:,end), [aperr(:,4:6),normv(aperr(:,4:6))]); xygo('lever');
        subplot(212), plot(lv(:,end), [lv(:,1:3),normv(lv(:,1:3))]); xygo('lever');
        title(sprintf('L_x = %.3f, L_y = %.3f, L_z = %.3f (m)', lever(1),lever(2),lever(3)));
    end

