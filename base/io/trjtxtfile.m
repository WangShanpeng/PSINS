function [imu, avp, gps] = trjtxtfile(fname, imu, avp, gps)
% Create & load PSINS text-format TRJ file.
%
% Prototype: [imu, avp, gps] = trjtxtfile(fname, imu, avp, gps)
% Inputs: fname - file name for write
%         imu - =[gyro,acc,t], 
%         avp - =[att,vn,pos,t],
%         gps - =[vn,pos,t],
% Outputs: imu, avp, gps
%
% See also  imufile, binfile, trjfile.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/09/2020
global glv
    if nargin==1  % load
       dd = load(fname);
       imu = [dd(:,2:4)*0.1*glv.sec, dd(:,5:7)*100*glv.ug, dd(:,1)]; % imuplot(imu)
       avp = [dd(:,8:10)*glv.deg, dd(:,11:13), dd(:,14:15)*glv.deg, dd(:,[16,1])]; % avplot(avp);
       gps = no0([dd(:,11:13), dd(:,14:15)*glv.deg, dd(:,[16,1])]); % gpsplot(gps);
       return;
    end
    %% write
    if nargin>3,
        avp = combinedata(avp(:,[1:3,end]), gps);
        avp = avp(:,[1:3,5:10,4]);
    end
    trj = combinedata(imu, avp);
    trj = trj(:,[1:6,8:end-1,7]);
    trj(:,1:3) = diff(round(cumsum([[0,0,0];trj(:,1:3)])/(0.1*glv.sec)),1);
    trj(:,4:6) = diff(round(cumsum([[0,0,0];trj(:,4:6)])/(100*glv.ug)),1);
    trj(:,[7:9,13:14]) = trj(:,[7:9,13:14])/glv.deg;
	fid = fopen(fname, 'wt');
	fprintf(fid, '%% PSINS-format TRJ log file. (DO NOT MODIFY!)\n');
	fprintf(fid, '%% --- Load Usage --------------------------\n');
	fprintf(fid, '%% glvs;\n');
	fprintf(fid, '%% dd = load(this_file_name);\n');
	fprintf(fid, '%% imu = [dd(:,2:4)*0.1*glv.sec, dd(:,5:7)*100*glv.ug, dd(:,1)]; imuplot(imu);\n');
	fprintf(fid, '%% avp = no0([dd(:,8:10)*glv.deg, dd(:,11:13), dd(:,14:15)*glv.deg, dd(:,[16,1])], 3); insplot(avp(:,[1:3,end]));\n');
	fprintf(fid, '%% gps = no0(avp(:,4:end), 4); gpsplot(gps);\n');
	fprintf(fid, '%% --- Usage End ---------------------------\n\n');
    for k=1:length(trj)
        fprintf(fid, '%.3f, %d,%d,%d, %d,%d,%d, ', trj(k,[end,1:6])');
        if abs(trj(k,9))>1e-6
            fprintf(fid, '%.4f,%.4f,%.3f, ', trj(k,7:9)');
        else
            fprintf(fid, '0,0,0, ');
        end
        if abs(trj(k,13))>1e-6
            fprintf(fid, '%.3f,%.3f,%.3f, %.8f,%.8f,%.3f\n', trj(k,10:15)');
        else
            fprintf(fid, '0,0,0, 0,0,0\n');
        end
    end
    fclose(fid);
    
    
    
