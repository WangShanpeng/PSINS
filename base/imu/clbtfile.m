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
            if isfield(clbt,'sf'), disp('sf'); disp(clbt.sf'); end
            if isfield(clbt,'Kg'), disp('Kg'); disp(clbt.Kg); end
            if isfield(clbt,'eb'), disp('eb(dph)'); disp(clbt.eb'/glv.dph); end
            if isfield(clbt,'Ka'), disp('Ka'); disp(clbt.Ka); end
            if isfield(clbt,'db'), disp('db(ug)'); disp(clbt.db'/glv.ug); end
            if isfield(clbt,'Ka2'), disp('Ka2(ug/g2)'); disp(clbt.Ka2'/glv.ugpg2); end
            if isfield(clbt,'Kap'), disp('Kap(ppm)'); disp(clbt.Kap'/glv.ppm); end
            if isfield(clbt,'rx'), disp('rx,ry,rz(cm)'); disp([clbt.rx';clbt.ry';clbt.rz']*100); end
            if isfield(clbt,'tGA'), disp('tGA(ms)'); disp(clbt.tGA*1000); end
            if isfield(clbt,'gSens'), disp('gSens(dphpg)'); disp(clbt.gSens/glv.dphpg); end
        else
            fid = fopen(fname, 'w');
            if strcmp(fname(end-3:end),'.bin')==1 % write to bin file
                data = [reshape(clbt.Kg',9,1);clbt.eb;reshape(clbt.Ka',9,1);clbt.db;clbt.Ka2;clbt.Kap;clbt.rx;clbt.ry;clbt.rz;clbt.tGA];
%                 if isfield(clbt,'sf'),  data=[clbt.sf; data]; end
                fwrite(fid, data, 'double');
            else % write to text file
                if isfield(clbt,'sf'),
                    fprintf(fid, 'sf = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e \t%.8e \t%.8e \t%.8e\r\n\t', clbt.sf'); fprintf(fid, '];\r\n');
                end
                fprintf(fid, 'Kg = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Kg'); fprintf(fid, '];\r\n');
                fprintf(fid, 'eb = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.eb/glv.dph); fprintf(fid, ']; \t%% deg/h\r\n');
                fprintf(fid, 'Ka = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Ka'); fprintf(fid, '];\r\n');
                fprintf(fid, 'db = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.db/glv.ug); fprintf(fid, ']; \t%% ug\r\n');
                fprintf(fid, 'Ka2 = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Ka2/glv.ugpg2); fprintf(fid, ']; \t%% ug/g^2\r\n');
                fprintf(fid, 'Kap = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.Kap/glv.ppm); fprintf(fid, ']; \t%% ppm\r\n');
                fprintf(fid, 'rx = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.rx*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'ry = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.ry*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'rz = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.rz*100); fprintf(fid, ']; \t%% cm\r\n');
                fprintf(fid, 'tGA = [\r\n\t');   fprintf(fid, '%.8e\r\n\t', clbt.tGA*1000); fprintf(fid, ']; \t%% ms\r\n');
                if isfield(clbt,'gSens'),
                    fprintf(fid, 'gSens = [\r\n\t');   fprintf(fid, '%.8e \t%.8e \t%.8e\r\n\t', clbt.gSens'/glv.dphpg); fprintf(fid, '];\r\n');
                end
            end
            fclose(fid);
        end
    else % in
        fname = clbt; clbt = [];
        if strcmp(fname(end-3:end),'.bin')==1 % read from bin file
            cbt = binfile(fname,1);
            clbt.Kg = reshape(cbt(1:9),3,3)';  clbt.eb = cbt(10:12);
            clbt.Ka = reshape(cbt(13:21),3,3)';  clbt.db = cbt(22:24);  clbt.Ka2 = cbt(25:27);  clbt.Kap = cbt(28:30);
            clbt.rx = cbt(31:33); clbt.ry = cbt(34:36); clbt.rz = cbt(37:39);
            clbt.tGA = cbt(40);
        elseif strcmp(fname(end-1:end),'.m')==1
            run(fname);
            clbt.Kg = Kg;  clbt.eb = eb';
            clbt.Ka = Ka;  clbt.db = db';  clbt.Ka2 = Ka2';  clbt.Kap = Kap';
            clbt.rx = rx'; clbt.ry = ry'; clbt.rz = rz';
            clbt.tGA = tGA;
        end
        clbtout = clbt;
    end
