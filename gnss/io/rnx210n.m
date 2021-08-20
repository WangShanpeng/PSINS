function eph = rnx210n(nFileName)
% A simple version of read RINEX-format (Ver2.10) navigation message file.
%
% Prototype: eph = rnx210n(nFileName)
% Inputs: nFileName - RINEX-format GPS navigation file name
% Outputs: eph - ephemeris records in struct array
%
% See also  rnx210o, findEph.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/08/2013, 05/07/2015
    fid = fopen(nFileName,'rt');
    s = fgetl(fid);
    if ~strfind(s,'NAVIGATION DATA')
        error('can not find the beginning of header!\r\n');
    end
    for lineIndex=2:100
        s = fgetl(fid); s(length(s)+1:81) = ' ';
        if strfind(s,'END OF HEADER')
            break;
        end 
    end
    %%
    idx = 1;
    c1 = 4:22; c2 = 23:41; c3 = 42:60; c4 = 61:79;
    eph = zeros(1000,31);
    while 1
        s1=fgetl(fid); s2=fgetl(fid); s3=fgetl(fid); s4=fgetl(fid); 
        s5=fgetl(fid); s6=fgetl(fid); s7=fgetl(fid); s8=fgetl(fid);
        if feof(fid),  break;   end
        if idx>length(eph)
            eph = [eph; zeros(1000,31)];
        end
        PRN = str2num(s1(1:2));
        [Toc, wn] = cal2gpst(str2num(s1(3:22)));
        eph(idx,:) = [PRN, Toc, ...
            str2num(...
            [          s1(c2),' ',s1(c3),' ',s1(c4),' ',...
            s2(c1),' ',s2(c2),' ',s2(c3),' ',s2(c4),' ',...
            s3(c1),' ',s3(c2),' ',s3(c3),' ',s3(c4),' ',...
            s4(c1),' ',s4(c2),' ',s4(c3),' ',s4(c4),' ',...
            s5(c1),' ',s5(c2),' ',s5(c3),' ',s5(c4),' ',...
            s6(c1),' ',s6(c2),' ',s6(c3),' ',s6(c4),' ',...
            s7(c1),' ',s7(c2),' ',s7(c3),' ',s7(c4),' ',...
            s8(c1),' ',s8(c2)])  ];           
        idx = idx + 1;
    end
    fclose(fid);
    %% delete redundance and sort
    eph(idx:end,:) = [];
    [junk, I] = unique(eph(:,1)*1e7+eph(:,2));
    eph = eph(I,:);
