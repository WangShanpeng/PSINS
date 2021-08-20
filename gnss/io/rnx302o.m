function [obs, head] = rnx302o(oFileName, nRecord)
% A simple version of read RINEX-format (Ver3.02) observation data file.
%
% Prototype: [obs, head] = rinexReadO(oFileName, nRecord)
% Inputs: oFileName - RINEX-format GPS observation file name
%         nRecord - first n records to be readed
% Outputs: obss - observation records in struct
%          obsTpIdx - observation record index = observation time array [tp]
%          head - obs file header information
%
% See also  rnx302n, rnx210n, rnx210o, obsEphLink.

% Originated by Yangling 2008/3/25, Tongji Unv.,China
% Modified by Yangongmin 16/09/2013, 05/07/2015, NWPU
%% read header
	fid = fopen(oFileName, 'rt');
    s = fgetl(fid);
    if ~strfind(s,'OBSERVATION DATA')
        error('can not find the beginning of header!\r\n');
    end
    head = []; otype = []; clumns = 0;
    for lines=2:300
        s = fgetl(fid); s = [s, ' '];
        if ~isempty(strfind(s,'# / OBS TYPES')) %观测值类型
            nn = str2num(s(5:6));  clumns = max(nn,clumns);
            if nn>13, s1 = fgetl(fid); s = [s(1:58),s1(7:end)]; end
            otype.WN=1; otype.tp=2; otype.PRN=3;
            for k=1:nn
                otype = setfield(otype, s([1,(k-1)*4+(8:10)]), 3+k);
            end
            head.otype = otype;
        elseif ~isempty(strfind(s,'END OF HEADER')) %头文件结束 
            break;
        end 
    end
%% reader obs record
    if nargin<2,  nRecord = 20000; end
    obs = [];  kobs = 1;  kmax = 0;
    ii = [17 33 49 65 81]; ii = [ii, ii(end)+(1:20)*16];
    for k=1:nRecord
        s = [fgetl(fid),' '];   if feof(fid),  break;  end
        if s(1)=='>'
            [sow,wn] = cal2gpst(str2num(s(3:30)));
            nn = str2num(s(34:35));
            if kobs+nn>kmax, obs = [obs; zeros(10000,3+clumns)]; kmax=kmax+10000; end
            for kk=1:nn
                s = fgetl(fid);
                sys = strfind('GREJCSM',s(1));  if isempty(sys), break; end
                PRN = str2num(s(2:3));
                obs(kobs,1:3) = [wn,sow,sys*100+PRN];
                nnn = fix(length(s)/16);
                for ki=1:nnn
                    val = str2num(s(ii(ki)-12:ii(ki)));
                    if isempty(val) val=0; end
                    obs(kobs,3+ki) = val;
                end
                kobs = kobs + 1;
            end
        end
    end
    obs(kobs:end,:) = [];
