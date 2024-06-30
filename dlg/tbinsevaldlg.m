function varargout = tbinsevaldlg(varargin)
% TBINSEVALDLG MATLAB code for tbinsevaldlg.fig
%      TBINSEVALDLG, by itself, creates a new TBINSEVALDLG or raises the existing
%      singleton*.
%
%      H = TBINSEVALDLG returns the handle to a new TBINSEVALDLG or the handle to
%      the existing singleton*.
%
%      TBINSEVALDLG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TBINSEVALDLG.M with the given input arguments.
%
%      TBINSEVALDLG('Property','Value',...) creates a new TBINSEVALDLG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before tbinsevaldlg_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to tbinsevaldlg_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help tbinsevaldlg

% Last Modified by GUIDE v2.5 01-Dec-2021 01:23:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @tbinsevaldlg_OpeningFcn, ...
                   'gui_OutputFcn',  @tbinsevaldlg_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before tbinsevaldlg is made visible.
function tbinsevaldlg_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to tbinsevaldlg (see VARARGIN)

% Choose default command line output for tbinsevaldlg
handles.output = hObject;
movegui(gcf,'center');
% Update handles structure
guidata(hObject, handles);
init_gui(hObject, handles);

% UIWAIT makes tbinsevaldlg wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function init_gui(hObject, handles)
glvs
if ~isfield(handles,'setting')
    setting.phie=1; setting.phin=1; setting.phiu=10; setting.dve=0; setting.dvn=0; setting.dvu=0; setting.dlat=1; setting.dlon=1; setting.dhgt=1;
    setting.ebx=10; setting.eby=10; setting.ebz=10; setting.dbx=1000; setting.dby=1000; setting.dbz=1000;
    setting.dkgxx=100; setting.dkgyx=10; setting.dkgzx=10; setting.dkgxy=10; setting.dkgyy=100; setting.dkgzy=10; setting.dkgxz=10; setting.dkgyz=10; setting.dkgzz=100;
    setting.dkaxx=100; setting.dkayx=10; setting.dkazx=10; setting.dkayy=100; setting.dkazy=10; setting.dkazz=100;
    setting.ka2x=10; setting.ka2y=10; setting.ka2z=10;
    setting.webx=1; setting.weby=1; setting.webz=1; setting.wdbx=100; setting.wdby=100; setting.wdbz=100;
    handles.setting = setting;
end
setting = handles.setting;
names = fieldnames(setting);
for k=1:length(names)
    if isempty(getfield(setting,names{k})), continue; end
    if isfield(handles,names{k}), set(getfield(handles,names{k}),'String',getfield(setting,names{k})); end
end
guidata(hObject, handles);
axes(handles.axes1); cla;

function bgcwhite(hObject)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function res = settingchk(hObject, handles, strfield)
setting = handles.setting;
val = str2double(get(hObject,'String'));
if isnan(val)
    uiwait(msgbox('请输入一数值！','modal'));
    set(hObject,'String',getfield(setting,strfield)); res = 0; return;
end
setting = setfield(setting,strfield,val);
handles.setting = setting;
guidata(hObject, handles);
updateaxis(handles);
res = 1;

function varargout = tbinsevaldlg_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function ebx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function ebx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'ebx');
function eby_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function eby_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'eby');
function ebz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function ebz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'ebz');
function dbx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dbx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dbx');
function dby_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dby_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dby');
function dbz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dbz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dbz');
function webx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function webx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'webx');
function weby_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function weby_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'weby');
function webz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function webz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'webz');
function wdbx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function wdbx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'wdbx');
function wdby_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function wdby_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'wdby');
function wdbz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function wdbz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'wdbz');
function phie_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function phie_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'phie');
function phin_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function phin_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'phin');
function phiu_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function phiu_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'phiu');
function dve_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dve_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dve');
function dvn_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dvn_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dvn');
function dvu_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dvu_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dvu');
function dlat_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dlat_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dlat');
function dlon_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dlon_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dlon');
function dhgt_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dhgt_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dhgt');

function dkgxx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgxx');
function dkgxx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgyx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgyx');
function dkgyx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgzx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgzx');
function dkgzx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgxy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgxy');
function dkgxy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgyy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgyy');
function dkgyy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgzy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgzy');
function dkgzy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgxz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgxz');
function dkgxz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgyz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgyz');
function dkgyz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkgzz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkgzz');
function dkgzz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkaxx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkaxx');
function dkaxx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkayx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkayx');
function dkayx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkazx_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkazx');
function dkazx_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkayy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkayy');
function dkayy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkazy_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkazy');
function dkazy_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function dkazz_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'dkazz');
function dkazz_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);

function ka2x_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function ka2x_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'ka2x');
function ka2y_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function ka2y_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'ka2y');
function ka2z_CreateFcn(hObject, eventdata, handles)
bgcwhite(hObject);
function ka2z_Callback(hObject, eventdata, handles)
settingchk(hObject, handles, 'ka2z');

function savesetting_Callback(hObject, eventdata, handles)
[filename, pathname] = uiputfile( {'tbipara*.mat'}, '保存参数');
if isequal(filename,0), return; end
cd(pathname);
setting = handles.setting;
save([pathname,filename],'setting');

function loadsetting_Callback(hObject, eventdata, handles)
[filename, pathname] = uigetfile( {'tbipara*.mat'}, '读取参数');
if isequal(filename,0), return; end
cd(pathname);
load([pathname,filename]);
handles.setting = setting;
init_gui(hObject, handles);
guidata(hObject, handles);
updateaxis(handles);

function loadtrj_Callback(hObject, eventdata, handles)
[filename, pathname] = uigetfile( {'*.mat'}, '加载轨迹');
if isequal(filename,0), return; end
load([pathname,filename]);
avperr = avperrset([1;1;10], 0.01, 1);
imuerr = imuerrset(0.1,100,0.01,10,  0,0,0,0,  100,100, 10,10, 10);
[xkpk, kfs, trj] = tbinseval(ap, avperr, imuerr, 0);
handles.trj = trj; handles.kfs = kfs;
guidata(hObject, handles);
updateaxis(handles);

function updateaxis(handles)
global glv
s=handles.setting;
if ~isfield(handles,'kfs'); return; end
pqt=handles.kfs.pqt; err0=handles.kfs.aierr; trj=handles.trj;
eth = earth(trj.avp(1,7:9)');
err1 = [s.phie*glv.min; s.phin*glv.min; s.phiu*glv.min; s.dve; s.dvn; s.dvu; s.dlat/eth.RMh; s.dlon/eth.clRNh; s.dhgt; ...
    s.ebx*glv.dph; s.eby*glv.dph; s.ebz*glv.dph; s.dbx*glv.ug; s.dby*glv.ug; s.dbz*glv.ug; ...
    s.dkgxx*glv.ppm; s.dkgyx*glv.sec; s.dkgzx*glv.sec; s.dkgxy*glv.sec; s.dkgyy*glv.ppm; s.dkgzy*glv.sec; s.dkgxz*glv.sec; s.dkgyz*glv.sec; s.dkgzz*glv.ppm; ...
    s.dkaxx*glv.ppm; s.dkayx*glv.sec; s.dkazx*glv.sec; s.dkayy*glv.ppm; s.dkazy*glv.sec; s.dkazz*glv.ppm; ...
    s.ka2x*glv.ugpg2; s.ka2y*glv.ugpg2; s.ka2z*glv.ugpg2; ...
    s.webx*glv.dpsh; s.weby*glv.dpsh; s.webz*glv.dpsh;    s.wdbx*glv.ugpsHz; s.wdby*glv.ugpsHz; s.wdbz*glv.ugpsHz ];
for k=1:9,  pqt(k,1:39) = err1'./err0'.*pqt(k,1:39); pqt(k,end)=norm(pqt(k,1:39));   end  % scale error
axes(handles.axes1); cla;
xtl = {'phiE/N/U', 'dVE/N/U', 'dLat/Lon/Hgt', 'ebx/y/z', 'dbx/y/z', 'dkgx/y/zx', 'dkgx/y/zy', 'dkgx/y/zz', 'dkax/y/zx', 'dkay/zy,zz', 'Ka2x/y/z', 'wgx/y/z', 'wax/y/z', 'Total'};
plot(pqt(7:9,:)','-.o','linewidth',2); xlim([1,40]); grid on; ylabel('\deltaP / m'); legend('\deltaL', '\delta\lambda', '\deltaH', 'Location','Best');
title(sprintf('T = %.3f (s);  Distance = %.3f (m);  CEP50 = %.3f (m)',  trj.avp(end,end)-trj.avp(1,end), norm(pp2vn(trj.avp(end,7:9)',trj.avp(1,7:9)',1)), sum(pqt(7:8,end))*0.59));
set(gca, 'xtick', [2:3:40,40], 'XTicklabel', xtl);
