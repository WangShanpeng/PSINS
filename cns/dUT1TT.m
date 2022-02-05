function [dUT1, dTT] = dUT1TT(dut1, dTAI)
% time diff calculation.
% dUT1 = UT1-UTC  % https://datacenter.iers.org/singlePlot.php?plotname=BulletinA_LatestVersion-UT1-UTC&id=6
% dTAI = TAI-UTC  % https://hpiers.obspm.fr/eoppc/bul/bulc/Leap_Second_History.dat

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/11/2021
    if nargin<2, dTAI=37; end
    if nargin<1, dut1=-0.5; end
    dUT1 = dut1; % UT1-UTC
    dTT = dTAI+32.184; % TT-UTC

    