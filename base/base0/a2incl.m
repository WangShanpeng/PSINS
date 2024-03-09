function incl = a2incl(att)
% Convert Euler angle to incline angle.
%
% Prototype: incl = a2incl(att)
% Input: att - =[pitch, roll, yaw, t]
% Output: incl - = incl angle & incl yaw angle
%
% See also  a2attr.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/12/2023
    if size(att,2)==1
        Cnb = a2mat(att);
        incl = [acos(Cnb(3,3)); -atan2(Cnb(1,3),Cnb(2,3))];
    else  % batch processing
        Cnb = a2matBatch(att);
        incl = [ acos(Cnb(:,9)), -atan2(Cnb(:,3),Cnb(:,6)) ];
        if size(att,2)>3
            incl = [incl(:,[1,1,2]), att(:,4:end)];
        end
    end
    