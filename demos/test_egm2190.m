glvs;
exedir = 'D:\ygm2018(已备份)\惯导与滤波\egm\重力场模型计算软件\';

%% (1) get pos
pos = ins(1:100:end,[7:9,16]);  pos0=pos(1,1:3)';
for k=2:length(pos)
    if norm(pp2vn(pos0, pos(k,1:3)', 1))<100,  pos(k,:) = 0;
    else,  pos0 = pos(k,1:3)';  end
end
pos = no0(pos,1);
fid = fopen([exedir,'posdata.txt'],'w');
fprintf(fid, '%.8f %.8f %.3f\n', [pos(:,2)/glv.deg, pos(:,1)/glv.deg, pos(:,3)]');
fclose(fid);

%% (2) run egmcal.exe
% goto run 'exedir\egmcal.exe' ...

%% (3) plot gn
gndata = load([exedir,'gndata.txt']);
gndata = [pos2dxyz(pos(:,1:3)), gndata(:,5:7), pos(:,end)];
myfig;
subplot(221), plot(gndata(:,end), gndata(:,4)/glv.ug); xygo('g^n_E / ug');
subplot(223), plot(gndata(:,end), gndata(:,5)/glv.ug); xygo('g^n_N / ug');
subplot(2,2,[2,4]), quiver(gndata(:,1), gndata(:,2), gndata(:,4), gndata(:,5)); xygo('X/m', 'Y/m');  axis equal