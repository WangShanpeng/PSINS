function bias = gabias(gbias, abias)
% Gyro & Acc constant bias setting.
%
% Prototype: Gyro & Acc constant bias setting
% Inputs: gbias - gyro bias, in deg/h
%         abias - acc bias, in ug
% Output: bias - = [gbias; abias]
% 
% See also  imuerrset, avpset, insupdate.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2021
global glv
    bias = [rep3(gbias)*glv.dph; rep3(abias)*glv.ug];
