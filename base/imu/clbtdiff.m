function [err, Kg, eb, Ka, db] = clbtdiff(Kga1, Kga2, gyrofirst)
% IMU calibration difference between Kga1, Kga2.
%
% Prototype: err = clbtdiff(Kga1, Kga2, gyrofirst)
% Inputs: Kga1 - calibration parameter array [Kg; eb'; Ka; db'];
%         Kga2 - same as Kga1
%         gyrofirst - =1 for gyro-first, =0 for acc-first
% Outputs: err - calibration difference
%          Kg,eb,Ka,db - calibration parameters from IMU1 to IMU2
%
% Example:
%     [err, Kg,eb,Ka,db] = clbtdiff(Kga1, Kga2);
%     imu2 = imuclbt(imu1, Kg,eb,Ka,db);
%
% See also  imuclbt, sysclbt.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/03/2021
global glv
    if nargin<3, gyrofirst=0; end
    if isstruct(Kga1), Kga1=[Kga1.Kg;Kga1.eb';Kga1.Ka;Kga1.db']; Kga2=[Kga2.Kg;Kga2.eb';Kga2.Ka;Kga2.db']; end
    if size(Kga1,1)==6,  % Kga=[Kg; Ka]
        Kga1=[Kga1(1:3,:);zeros(1,3); Kga1(4:6,:);zeros(1,3)];
        Kga2=[Kga2(1:3,:);zeros(1,3); Kga2(4:6,:);zeros(1,3)]; 
    end
    Kg1=Kga1(1:3,:); eb1=Kga1(4,:); Ka1=Kga1(5:7,:); db1=Kga1(8,:);
    Kg2=Kga2(1:3,:); eb2=Kga2(4,:); Ka2=Kga2(5:7,:); db2=Kga2(8,:);
    if gyrofirst==1,
        dK = Kg2*Kg1^-1;
    else
        dK = Ka2*Ka1^-1;
    end
    C = svdest(dK);
    errKg = C'*Kg2*Kg1^-1 - eye(3);
    errKa = C'*Ka2*Ka1^-1 - eye(3);
    err = [errKg; eb2-eb1; errKa; db2-db1];
    errKg, erreb=(eb2-eb1)/glv.dph, errKa, errdb=(db2-db1)/glv.ug,
    if nargout>1
        Kg = Kg2*Kg1^-1;  eb = (eb2-eb1)';
        Ka = Ka2*Ka1^-1;  db = (db2-db1)';
    end
    return;
    % example:
    Kga1 = [eye(3); eye(3)];
    Kga2 = [ [1       -0.0001   0.0002;
             0.0001   1        0.0003;
             -0.0002  -0.0003  1.001 ];
             eye(3);
             ];
    [err, Kg,eb,Ka,db] = clbtdiff(Kga1, Kga2)
