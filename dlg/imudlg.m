function varargout = imudlg(varargin)
%% IMUDLG MATLAB code for imudlg.fig. DO NOT EDIT!!!
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @imudlg_OpeningFcn, ...
                   'gui_OutputFcn',  @imudlg_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end
if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function imudlg_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
movegui(gcf,'center');
init_gui(handles);

function varargout = imudlg_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
%%----------------------------------------------------------------------

function init_gui(handles, setting0)
global setting dd
glvs
if ~exist('setting0', 'var')
    setting.filename = 'example.dat';
    set(handles.filename,'String',setting.filename);
    setting.reviewrow = 20;
    setting.recordclm = 7;
    setting.canntload = 1;
    setting.time = 7; setting.timets = 0.01;
    setting.gyrox = 1; setting.gyroy = 2; setting.gyroz = 3;
    setting.accx = 4; setting.accy = 5; setting.accz = 6;
    setting.latitude = 34.25; setting.longitude = 108.91; setting.height = 380;
    setting.pitch = 0; setting.roll = 0; setting.yaw = 0;
    setting.gyroscale = 1.0;  setting.gyrounit = 1; setting.accscale = 1.0;  setting.accunit = 1; setting.g0 = 9.78033;
    setting.imu = imustatic([zeros(6,1);posset(setting.latitude,setting.longitude,setting.height)], setting.timets, 100, imuerrset(0.01,100,0.001,10));
    setting.t0 = setting.imu(1,end); setting.t1 = setting.imu(end,end); setting.recordcount = length(setting.imu); setting.recordtime = setting.t1;
else
    names = fieldnames(setting);
    for k=1:length(names)
        val = getfield(setting0,names{k});
        if ~isempty(val), setting = setfield(setting,names{k},val); end
    end
end
names = fieldnames(setting);
for k=1:length(names)
    if isempty(getfield(setting,names{k})), continue; end
    if isfield(handles,names{k}), set(getfield(handles,names{k}),'String',getfield(setting,names{k})); end
end
set(handles.time,'String',setting.timets); set(handles.tscheck,'Value',1);
set(handles.direction, 'String', ['R-F-U';'F-R-D';'F-U-R';'F-L-U'; 'B-L-D']);
set(handles.axisselect, 'String', ['GyroX';'GyroY';'GyroZ';'AccX ';'AccY ';'AccZ ']);
set(handles.gyrounit, 'String', ['rad  ';'deg  ';'sec  ';'rad/s';'deg/s']); set(handles.gyrounit, 'Value', setting.gyrounit);
set(handles.accunit, 'String', ['m/s ';'g0*s';'ug*s';'m/ss';'g0  ']); set(handles.accunit, 'Value', setting.accunit);
dd = [];

function bgcwhite(hObject)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function res = settingchk(hObject, handles, strfield)
global setting
val = str2double(get(hObject,'String'));
if isnan(val)
    uiwait(msgbox('请输入一数值！','modal'));
    set(hObject,'String',getfield(setting,strfield)); res = 0; return;
end
setting = setfield(setting,strfield,val);
res = 1;
function [k0, k1] = t0t1check(t)
global setting
k0 = find(t>=setting.t0, 1)+1; k1 = find(t>=setting.t1, 1);
if isempty(k0), k0=0; k1=0; end
if isempty(k1), k0=0; k1=0; end
if k0>=k1, k0=0; k1=0; uiwait(msgbox('t0 和 t1 设置错误！','modal')); end

function filereview_Callback(hObject, eventdata, handles)
function filereview_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function reviewrow_Callback(hObject, eventdata, handles)
if settingchk(hObject, handles, 'reviewrow'), filereview(handles); end
function reviewrow_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function recordclm_Callback(hObject, eventdata, handles)
function recordclm_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function tscheck_Callback(hObject, eventdata, handles)
global setting
if get(hObject,'Value')==1, set(handles.time,'String',setting.timets);
else  set(handles.time,'String',setting.time); end
function time_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'time');
function time_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function gyrox_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'gyrox');
function gyrox_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function gyroy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'gyroy');
function gyroy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function gyroz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'gyroz');
function gyroz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function gyroscale_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'gyroscale');
function gyroscale_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function accx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'accx');
function accx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function accy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'accy');
function accy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function accz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'accz');
function accz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function accscale_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'accscale');
function accscale_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function gyrounit_Callback(hObject, eventdata, handles)
global setting
setting.gyrounit = get(handles.gyrounit,'Value');
function gyrounit_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function accunit_Callback(hObject, eventdata, handles)
global setting
setting.accunit = get(handles.accunit,'Value');
function accunit_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function g0_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'g0');
function g0_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function recordcount_Callback(hObject, eventdata, handles)
function recordcount_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function recordtime_Callback(hObject, eventdata, handles)
function recordtime_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function direction_Callback(hObject, eventdata, handles)
function direction_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function t0_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 't0');
function t0_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function t1_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 't1');
function t1_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function axisselect_Callback(hObject, eventdata, handles)
function axisselect_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function latitude_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'latitude');
function latitude_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function longitude_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'longitude');
function longitude_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function height_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'height');
function height_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function pitch_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'pitch');
function pitch_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function roll_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'roll');
function roll_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function yaw_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'yaw');
function yaw_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);

function filereview(handles)
global setting
if ~exist(setting.filename,'file'), return; end
fid = fopen(setting.filename, 'r');
row = setting.reviewrow; clm = 512;
tls = repmat(' ',row,clm); clm = 0;
setting.canntload = 0;
for kk=1:row-1
    tline = fgetl(fid); if feof(fid), break; end
    len = length(tline); clm = max(clm,len);
    if len==0, len = 1; tline = ' '; end
    for k=1:length(tline)
        if tline(k)==9, tline(k)=' '; end  % '\t' -> ' '
    end
    tls(kk,1:len) = tline;
    tln = deblank(tline);  if ~isempty(tln), if tln(1)~='%', if isempty(str2num(tln)), setting.canntload = 1; end; end; end
end
tline = [' ',tline];
idx = 0;
for k=2:len+1
    p = tline(k-1); c = tline(k);
    if c=='e'||c=='E'||c=='d'||c=='D'
        tline(k+2) = 'x'; k = k+3; continue;
    end
    if (p==' '|| p==9 ||p==','||p==';'||p=='+'||p=='-') && (c>='0'&&c<='9')
        idx = idx+1; tline(k) = sprintf('%d',mod(idx,10));
    else
        if c>='0'&&c<='9', tline(k) = 'x'; else tline(k) = c; end
    end
end
tls(kk+1,1:len) = tline(2:end);
fclose(fid);
setting.recordclm = idx;  setting.recordcount = 0;
set(handles.filereview,'String',tls(1:kk+1, 1:clm));
set(handles.recordclm,'String',idx);

function fileselect_Callback(hObject, eventdata, handles)
global setting
[filename, pathname] = uigetfile( {'*.txt;*.dat;*.imu','IMU Files (*.txt;*.dat;*.imu)'; '*.*',  'All Files (*.*)'}, '选择IMU文件');
if isequal(filename,0), return; end
cd(pathname);
fname = [pathname,filename];
set(handles.filename,'String',fname);  setting.filename = fname;
filereview(handles);
set(handles.recordcount,'String','?');

function load_Callback(hObject, eventdata, handles)
global setting dd
if exist(setting.filename,'file')==0, uiwait(msgbox('打开数据文件错误！','modal')); return; end
if setting.canntload==1, uiwait(msgbox('文件含非法数据行，请将其删除或用%注释后再导入！','modal')); return; end
if ~isequal(get(handles.recordcount,'String'),'?'), uiwait(msgbox('数据文件已导入！','modal')); return; end
h = msgbox('数据导入中......','modal');
dd = load(setting.filename);
close(h);
set(handles.recordcount,'String',size(dd,1));

function imuget_Callback(hObject, eventdata, handles)
global setting dd
if isempty(dd), uiwait(msgbox('请先导入数据文件！','modal')); return; end
idx = [setting.gyrox, setting.gyroy, setting.gyroz, setting.accx, setting.accy, setting.accz]; 
setting.imu = zeros(length(dd),7);
ts = str2double(get(handles.time,'String'));
if get(handles.tscheck,'Value')==0, setting.imu(:,7) = dd(:,ts); ts = mean(diff(setting.imu(:,7)));
else setting.imu(:,7) = (1:length(dd))'*ts; end
unit = setting.gyrounit;
if unit==1, gyrounit=1; elseif unit==2, gyrounit=pi/180; elseif unit==3, gyrounit=pi/180/3600; elseif unit==4, gyrounit=ts; elseif unit==5, gyrounit=ts*pi/180; end 
unit = setting.accunit;
if unit==1, accunit=1; elseif unit==2, accunit=setting.g0; elseif unit==3, accunit=setting.g0/1e6; elseif unit==4, accunit=ts; elseif unit==5, accunit=ts*setting.g0; end 
for k=1:6
    if k<=3, scale = setting.gyroscale*gyrounit; else scale = setting.accscale*accunit; end
    setting.imu(:,k) = sign(idx(k))*dd(:,abs(idx(k)))*scale;
end
setting.recordtime = setting.imu(end,7)-setting.imu(1,7)+ts; setting.timets = ts;
set(handles.recordtime, 'String', setting.recordtime);
contents = cellstr(get(handles.direction,'String'));
direction = contents{get(handles.direction,'Value')};
setting.imu = imurfu(setting.imu, direction(1:2:end));
imuplot(setting.imu);

function allan_Callback(hObject, eventdata, handles)
global setting
glvs
[k0, k1] = t0t1check(setting.imu(:,end)); if k0==0, return; end
val = get(handles.axisselect,'Value');
if val<=3, avar(setting.imu(k0:k1,val)/setting.timets/glv.dph, setting.timets);
else       avar(setting.imu(k0:k1,val)/setting.timets/glv.ug, setting.timets, '\itx \rm/ \mu\itg');  end

function align_Callback(hObject, eventdata, handles)
global setting
glvs
[k0, k1] = t0t1check(setting.imu(:,end)); if k0==0, return; end
pos = posset(setting.latitude, setting.longitude, setting.height);
att = aligni0vn(setting.imu(k0:k1,:), pos, 60);
setting.pitch = att(1)/glv.deg; setting.roll = att(2)/glv.deg; setting.yaw = att(3)/glv.deg;  
set(handles.pitch, 'String', setting.pitch); set(handles.roll, 'String', setting.roll); set(handles.yaw, 'String', setting.yaw);

function navigation_Callback(hObject, eventdata, handles)
global setting 
glvs
[k0, k1] = t0t1check(setting.imu(:,end)); if k0==0, return; end
att = [setting.pitch; setting.roll; setting.yaw]*glv.deg;
pos = posset(setting.latitude, setting.longitude, setting.height);
inspure(setting.imu(k0:k1,:), [att;zeros(3,1);pos], 'H');

function loadimu_Callback(hObject, eventdata, handles)
global setting
[filename, pathname] = uigetfile( {'*.mat'}, '直接导入IMU');
if isequal(filename,0), return; end
cd(pathname);
load([pathname,filename], 'imu');
if exist('imu','var'), 
    setting.imu = imu; imuplot(setting.imu);
    ts = diff(imu(1:2,end));
%     setting.t0 = setting.imu(1,end); setting.t1 = setting.imu(end,end); setting.recordcount = length(setting.imu);
    setting.recordtime = setting.imu(end,end)-setting.imu(1,end); setting.timets = ts; setting.recordcount = length(setting.imu);
    setting.filename = 'import IMU';
    init_gui(handles, setting);
else
    uiwait(msgbox('IMU文件格式无效！','modal'));
end

function saveimu_Callback(hObject, eventdata, handles)
global setting
[filename, pathname] = uiputfile( {'*.mat'}, '保存IMU', 'imu.mat');
if isequal(filename,0), return; end
cd(pathname);
imu = setting.imu;
save([pathname,filename], 'imu');

function loadsetting_Callback(hObject, eventdata, handles)
global setting
[filename, pathname] = uigetfile( {'*.cfg'}, '读取参数');
if isequal(filename,0), return; end
cd(pathname);
load([pathname,filename], '-mat', 'setting0');
if exist('setting0','var'), init_gui(handles, setting0);
else  uiwait(msgbox('参数文件格式无效！','modal'));  end

function savesetting_Callback(hObject, eventdata, handles)
global setting
[filename, pathname] = uiputfile( {'*.cfg'}, '保存参数', '_imudlg_setting.cfg');
if isequal(filename,0), return; end
cd(pathname);
setting0 = setting; setting0.imu = []; setting0.filename = [];
setting0.t0 = []; setting0.t1 = []; setting0.recordcount = []; setting0.recordtime = []; setting0.recordclm = [];
save([pathname,filename], 'setting0');
