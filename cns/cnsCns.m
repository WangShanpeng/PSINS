function Cns = cnsCns(qis, pos, Cie, t1, Cbs)
% In CNS application, to get n-navigation frame to star-sensor/body frame, C^n_s.
%
% Prototype: Cns = cnsCns(qis, pos, Cie, t01, Cbs)
% Inputs: qis - star-sensor quaternion
%         pos - position [lat, lon, hgt]
%         Cie - Cie at t0, using cnsCie to get it
%         t1 - time start from t0
%         Cbs - CNS install matrix C^b_s, from b-frame to s-frame
% Output: Cns - C^n_s/b tranformation matrix
%
% See also  cnsCie, Precmat, Nutmat, GAST.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/11/2021
global glv
    if length(qis)==3, qis=q32q4(qis); end
    Cns = (q2mat(qconj(qis))*Cie*rxyz(glv.wie*t1)*pos2cen(pos))';
    if exist('Cbs','var'), Cns = Cns*Cbs'; end
