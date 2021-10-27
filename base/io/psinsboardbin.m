function [imu, mag, bar, avp, gps, gs, temp, head, chksum] = psinsboardbin(lasti, bindir)
% Read bin-data saving form the PSINS-Board
% Example:
%     [imu, mag, bar, avp, gps, gs, temp, head, chksum] = psinsboardbin(0);
%     iter=fix(gs(:,1)); rat=(gs(:,1)-iter)*100;  myfig; subplot(211), plot(iter-100,rat,'*'); xygo('iter', 'CPU usage / %');
%     cnt1=fix(gs(:,2)); cnt0=(gs(:,2)-cnt1)*100;  subplot(212), plot(gs(:,end), [cnt0,cnt1, cnt1-cnt0],'*'); xygo('n');

global glv
%     if nargin<2, bindir=[glv.rootpath,'\vc60\data']; end
    if nargin<2, bindir='.'; end
    if nargin<1, lasti=0; end
    if ischar(lasti)
        fname = lasti;
    else
        fname = dir([bindir,'\PSINS*.bin']);
        fname = fname(end-lasti).name;      
    end
    fid = fopen(fname, 'rb');
    dd = fread(fid, [35, inf], 'float32')';
    fclose(fid);
    ts = 0.01;
    [data, idx] = reminct(dd(:,2)); dd=dd(idx,:);
    chksum = typecast(single(dd(:,35)),'uint32');
    dd = dd(chksum>0,:);
    head = typecast(single(dd(:,1)),'uint32');
    imu = norep([[dd(:,3:5)*glv.dps, dd(:,6:8)]*ts, dd(:,2)],1:6);
    mag = norep(dd(:,[9:11,2]),1:3);
    bar = norep(dd(:,[12,2]),1);
    avp = no0([dd(:,13:15)*glv.deg, dd(:,16:18), (dd(:,[21,19])+dd(:,[22,20]))*glv.deg, dd(:,[23,2])],1);
    gps = no0([dd(:,24:26), (dd(:,[29,27])+dd(:,[30,28]))*glv.deg, dd(:,[31,2])], 1);
    gs = dd(:,[32:33,2]);
    temp = dd(:,[34,2]);
    if nargout==0
        imuplot(imu);  magplot(mag);  baroplot(bar);
        if ~isempty(avp), insplot(avp); end
        if ~isempty(gps), gpsplot(gps); end
        templot(temp);
        imu = [];  % no disp
    end
    