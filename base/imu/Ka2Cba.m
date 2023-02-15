function [Cba, Sfa, Q] = Ka2Cba(Ka)
% IMU calibration matrix to installation matrix.
% Ref. '惯性仪器测试与数据分析' 10.2.1节
%
% Prototype: [Cba, Sfa, Q] = Ka2Cba(Ka)
% Input: Ka - calibration matrix
% Outputs: Cba - installation matrix
%          Sfa - Scale factors
%
% See also  imuclbt, Cba2Ka.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 04/02/2023
    iKa = Ka^-1;
    Sfa = normv(iKa);
    Cba = (diag(Sfa)^-1*iKa)';
    if nargout>2
        [Q, Cba] = qr(Cba);  % Q<0?
    end
    