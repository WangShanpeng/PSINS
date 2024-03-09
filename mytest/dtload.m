function [imu, od, ddimu, ddod] = dtload(dirstr,isfig)
global glv;
    ts = 1/125;
    % #GX:-0.24603271484375000000000#GY:-0.32275390625000000000000#GZ:-19.55090332031250000000000
    % #AX:+0.57846156673431400000000#AY:-5.51006807384491000000000#AZ:+8.10848765373230000000000
    % #IX:0.00000000000000000000000#IY:0.00000000000000000000000#IZ:0.00000000000000000000000#830,0
    if nargin<2, isfig=0; end
    if dirstr(end)~='\', dirstr=[dirstr,'\']; end
%     fname = [dirstr, 'IMU_Raw.txt'];
%     bytes = dir(fname);
%     len = fix(bytes.bytes/270);  ddimu = zeros(len,11);
%     fid = fopen(fname);
%     for k=1:len
%         tline = fgetl(fid);  if ~ischar(tline), break, end
%         if length(tline)<270, continue; end
%         ddimu(k,:) = sscanf(tline, '#GX:%f#GY:%f#GZ:%f #AX:%f#AY:%f#AZ:%f #IX:%f#IY:%f#IZ:%f #%f,%f');
%     end
%     ddimu=no0(ddimu,10);
%     fclose(fid);
    fid = fopen([dirstr, 'IMU_Raw.txt']);
    ddimu = fscanf(fid, '#GX:%f#GY:%f#GZ:%f#AX:%f#AY:%f#AZ:%f#IX:%f#IY:%f#IZ:%f#%f,%f\n');
    fclose(fid);
    len = floor(length(ddimu)/11);
    ddimu = reshape(ddimu(1:len*11,:),11,len)';
    imu = [[ddimu(:,1:3)*glv.dps, ddimu(:,4:6)]*ts, ddimu(:,end-1)/1000];
    %%
    % #0000000096\+00000.000\+00000.000\+00000.000
%     fid = fopen([dirstr, 'MILE.txt']);
%     ddod = zeros(len,4);
%     for k=1:len
%         tline = fgetl(fid);  if ~ischar(tline), break, end
%         if length(tline)<44, continue; end
%         ddod(k,:) = sscanf(tline, '#%f\\%f\\%f\\%f');
%     end
%     ddod=no0(ddod,1);
%     fclose(fid);
    fid1 = fopen([dirstr, 'MILE.txt']);
    ddod = fscanf(fid1, '#%f\\%f\\%f\\%f\n');
    fclose(fid1);
    len = floor(length(ddod)/4);
    ddod = reshape(ddod(1:len*4,:),4,len)';
    od = [diff(ddod(:,2:4)), ddod(2:end,1)/1000];
    %%
	if isfig,
        imuplot(imu);
        myfig, plot(od(:,end), cumsum(od(:,1:3))), xygo('dist / m');
    end

