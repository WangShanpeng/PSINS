% Transfer align trajector simulation for the vc60 Demo_CAligntf example.
% See also  test_align_transfer_imu_simu, test_align_transfer.
% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/04/2021
glvs
ts = 0.01;
vel0 = 180;
avp0 = avpset([0;0;-30], vel0, [34;108;380+1000]);
roll_f = 0.5;  % roll frequency
t = (0:ts:0.5/roll_f)';
roll = 20*glv.deg * [cos(2*pi*roll_f*t)-1; 
    -2*cos(2*pi*roll_f*t); 
    cos(2*pi*roll_f*t)+1] * 0.5;  % roll angle sequence
wy = diff(roll)/ts;
wt = [zeros(20/ts,3);       % uniform
    zeros(length(wy),1), wy, zeros(length(wy),1); % swaying
    zeros(10/ts,3) ];       % uniform
at = zeros(size(wt));
at(5/ts:15/ts,2) = 10; [b,a]=ar1coefs(ts,0.1); at(:,2)=filtfilt(b,a,at(:,2)); % accelerate
wat = [zeros(length(wt),2), wt, at];  
wat(:,1) = ts; wat(:,2) = vel0;
trj = trjsimu(avp0, wat, ts, 1);
avp = avplever(trj.avp, [1;2;0.5]);
avp(:,1:3) = aaddmu(avp(:,1:3), [10;20;30]*glv.min);  % avpcmpplot(trj.avp, avp, 'mu');
binfile('transtrj.bin', [trj.imu(3:end,1:6),avp(1:end-2,:)]);
% binfile('transtrj.bin', [trj.imu(:,1:6),avp]);
insplot(trj.avp);
imuplot(trj.imu);
