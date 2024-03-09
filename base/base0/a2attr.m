function attr = a2attr(att)
% Convert Euler angle to dual Euler angle.
%
% Prototype: attr = a2attr(att)
% Input: att - =[pitch, roll, yaw, t]
% Output: attr - = dual Euler angle
%
% See also  a2mat, a2matBatch, m2att, a2incl.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/10/2023
    if size(att,2)==1
        Cnb = a2mat(att);
        [att, attr] = m2att(Cnb);
    else  % batch processing
        Cnb = a2matBatch(att);
        attr = [ atan2(Cnb(:,8),Cnb(:,9)), asin(-Cnb(:,7)), atan2(Cnb(:,4),Cnb(:,1)) ];
        if size(att,2)>3
            attr = [attr, att(:,4:end)];
        end
    end
    