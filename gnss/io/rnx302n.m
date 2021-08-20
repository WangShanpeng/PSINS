function [eph, head] = rnx302n(nFileName)
% A simple version of read RINEX-format (Ver3.02) navigation message file.
%
% Prototype: [ephs, ephToeIdx] = rinexReadN(nFileName)
% Inputs: nFileName - RINEX-format GPS navigation file name
% Outputs: eph - ephemeris records in struct array
%          head - ephemeris header
%
% See also  rnx302o, rnx210n, rnx210o, epha2s, findEph.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013, 05/07/2015
    fid = fopen(nFileName,'rt');
    s = fgetl(fid);
    if ~strfind(s,'RINEX VERSION')
        error('can not find the beginning of header!\r\n');
    end
    head = [];
    for lineIndex=2:300
        s = fgetl(fid); s(length(s)+1:81) = ' ';
        if strfind(s,'GPSA')
            head.GPSA = str2num(s(6:56));
        elseif strfind(s,'GPSB')
            head.GPSB = str2num(s(6:56));
        elseif strfind(s,'BDSA')
            head.BDSA = str2num(s(6:56));
        elseif strfind(s,'BDSB')
            head.BDSB = str2num(s(6:56));
        elseif strfind(s,'END OF HEADER')
            break;
        end 
    end
    %%
    c1 = 5:23; c2 = 24:42; c3 = 43:61; c4 = 62:80;
    keph = 1;  kmax = 0;
    eph = [];
    while 1
        s1=fgetl(fid);
        if feof(fid), break; end
        if keph>kmax,  eph = [eph; zeros(1000,31)];  kmax=kmax+1000;  end
        sys = strfind('GREJCSM',s1(1));  if isempty(sys), break; end 
        PRN = str2num(s1(2:3));  sow = cal2gpst(str2num(s1(4:23)));  %s1(1:23),
        if sys==1||sys==5
            s2=fgetl(fid); s3=fgetl(fid); s4=fgetl(fid); s5=fgetl(fid); s6=fgetl(fid); s7=fgetl(fid); s8=fgetl(fid);
            eph(keph,:) = [ sys*100+PRN,sow, ...
                str2num([           s1(c2),' ',s1(c3),' ',s1(c4),' ',s2(c1),' ',s2(c2),' ',s2(c3),' ',s2(c4),' ',...
                         s3(c1),' ',s3(c2),' ',s3(c3),' ',s3(c4),' ',s4(c1),' ',s4(c2),' ',s4(c3),' ',s4(c4),' ',...
                         s5(c1),' ',s5(c2),' ',s5(c3),' ',s5(c4),' ',s6(c1),' ',s6(c2),' ',s6(c3),' ',s6(c4),' ',...
                         s7(c1),' ',s7(c2),' ',s7(c3),' ',s7(c4),' ',s8(c1),' ',s8(c2)]) ];
            keph = keph + 1;
        elseif sys==3||sys==4
            s2=fgetl(fid); s3=fgetl(fid); s4=fgetl(fid); s5=fgetl(fid); s6=fgetl(fid); s7=fgetl(fid); s8=fgetl(fid);
        elseif sys==2||sys==6
            s2=fgetl(fid); s3=fgetl(fid); s4=fgetl(fid);
            eph(keph,1:17) = [ sys*100+PRN,sow, ...
                str2num([           s1(c2),' ',s1(c3),' ',s1(c4),' ',s2(c1),' ',s2(c2),' ',s2(c3),' ',s2(c4),' ',...
                         s3(c1),' ',s3(c2),' ',s3(c3),' ',s3(c4),' ',s4(c1),' ',s4(c2),' ',s4(c3),' ',s4(c4),' ' ]) ];
            keph = keph + 1;
        end
   end
    fclose(fid);
    %% delete redundance and sort
    eph(keph:end,:) = [];
    [n, I] = unique(eph(:,1)*1e7+eph(:,2));
    eph = eph(I,:);

