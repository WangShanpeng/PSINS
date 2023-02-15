function Ka = Cba2Ka(Cba, Sfa)
% IMU installation matrix to calibration matrix.
% Ref. '惯性仪器测试与数据分析' 10.2.1节
%
% Prototype: Ka = Cba2Ka(Cba, Sfa)
% Inputs: Cba - installation matrix
%          Sfa - Scale factors
% Output: Ka - calibration matrix
%
% See also  imuclbt, Ka2Cba.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 04/02/2023
    Ka = inv(diag(Sfa)*(Cba'));
    