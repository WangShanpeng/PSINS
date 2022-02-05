function Cie = cnsCie(utc, s, dUT1, dTAI)
% In CNS application, J2000 to ECEF rotation matrix 'Cie' calculation
%
% Prototype: Cie = cnsCie(utc, s, dUT1, dTAI)
% Inputs: utc - UTC day in [year,month,day] array
%         s - UTC seconds within a day, [0~86400)
%         dUT1 - = UT1-UTC
%         dTAI - = TT-TAI
% Output: Cie - C^i_e matrix
%
% See also  dUT1TT, Precmat, Nutmat, GAST, GMST, JD, cnsCns.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/11/2021
    if nargin<4, dTAI=37; end
    if nargin<3, dUT1=-0.5; end
    if nargin<2, s=0; end
    [dUT1, dTT] = dUT1TT(dUT1, dTAI);
    [theta, ~, ~, TT, eps, dpsi, deps] = GAST(JD(utc), s+dUT1, dTT);
    NP = rxyz(eps+deps,'x') * rxyz(dpsi,'z') * rxyz(-eps,'x') * Precmat(TT);
    Cie = NP'*rxyz(theta,'z');