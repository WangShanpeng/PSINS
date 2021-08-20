function clbtout = clbtfile(clbt, fname)
% IMU calibration struct in/out.
%
% Prototype: clbt = clbtfile(clbt, fname)
% Inputs: clbt - calibration struct input
%         fname - file name
% Output: clbtout - calibration struct output
%
% See also  imuclbt, sysclbt.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/03/2020
global glv
    if nargout==0  % out
        if nargin==1 % display
            if isfield(clbt,'Kg'), disp('Kg'); disp(clbt.Kg); end
            if isfield(clbt,'eb'), disp('eb(dph)'); disp(clbt.eb'/glv.dph); end
            if isfield(clbt,'Ka'), disp('Ka'); disp(clbt.Ka); end
            if isfield(clbt,'db'), disp('db(ug)'); disp(clbt.db'/glv.ug); end
            if isfield(clbt,'Ka2'), disp('Ka2(ug/gg)'); disp(clbt.Ka2'/glv.ugpg2); end
            if isfield(clbt,'rx'), disp('rx,ry,rz(cm)'); disp([clbt.rx';clbt.ry';clbt.rz']*100); end
            if isfield(clbt,'tGA'), disp('tGA(ms)'); disp(clbt.tGA*1000); end
        else
            fid = fopen(fname, 'w');
            if strcmp(fname(end-3:end),'.bin')==1 % write to bin file
                fwrite(fid, [reshape(clbt.Kg',9,1);clbt.eb;reshape(clbt.Ka',9,1);clbt.db;clbt.Ka2;clbt.rx;clbt.ry;clbt.rz;clbt.tGA], 'double');
            else % write to text file
                fprintf(fid, 'Kg = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Kg'); fprintf(fid, '];\r\n');
                fprintf(fid, 'eb = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.eb/glv.dph); fprintf(fid, ']; \t%% deg/h\r\n');
                fprintf(fid, 'Ka = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Ka'); fprintf(fid, '];\r\n');
                fprintf(fid, 'db = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.db/glv.ug); fprintf(fid, ']; \t%% ug\r\n');
                fprintf(fid, 'Ka2 = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Ka2/glv.ugpg2); fprintf(fid, ']; \t%% ug/g^2\r\n');
                fprintf(fid, 'rx = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.rx*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'ry = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.ry*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'rz = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.rz*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'tGA = [\r\n\t');   fprintf(fid, '%.8e\r\n\t', clbt.tGA*1000); fprintf(fid, ']; \t%% ms\r\n');
            end
            fclose(fid);
        end
    else % in
        fname = clbt; clbt = [];
        if strcmp(fname(end-3:end),'.bin')==1 % read from bin file
            cbt = binfile(fname,1);
            clbt.Kg = reshape(cbt(1:9),3,3)';  clbt.eb = cbt(10:12);
            clbt.Ka = reshape(cbt(13:21),3,3)';  clbt.db = cbt(22:24);  clbt.Ka2 = cbt(25:27);
            clbt.rx = cbt(28:30); clbt.ry = cbt(31:33); clbt.rz = cbt(34:36);
            clbt.tGA = cbt(37);
        else
            NA = 1;
        end
        clbtout = clbt;
    end
