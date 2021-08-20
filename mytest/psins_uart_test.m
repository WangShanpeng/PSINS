% Copyright(c) 2009-2018, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/08/2018
global glb
res = instrfind('Type', 'serial');
if ~isempty(res)
    fclose(res(end)); delete(res(end));
end
glb.plotfirst = 1;
glb.plotbuf = zeros(200,35);
glb.hfig = figure;
glb.fid = fopen('test.bin','w');
glb.recbuflen = 35*4*20;
uart = serial('com4');  glb.uart = uart;
uart.baudrate = 460800;
uart.parity = 'none';
uart.stopbits = 1;
uart.inputbuffersize = glb.recbuflen;
uart.BytesAvailableFcnMode = 'byte';
uart.BytesAvailableFcnCount = glb.recbuflen;
uart.BytesAvailableFcn = @psinsuartcallback;
fopen(uart);
return

glvs; ts = 0.01;
fid = fopen('test.bin', 'rb');
dd = fread(fid, [35, inf], 'float32')';
fclose(fid);
imu = [[dd(:,3:5)*glv.dps, dd(:,6:8)]*ts, dd(:,2)]; imuplot(imu);

