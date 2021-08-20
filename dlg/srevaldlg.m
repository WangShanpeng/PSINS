function varargout = srevaldlg(varargin)
% SREVALDLG MATLAB code for srevaldlg.fig
%      SREVALDLG, by itself, creates a new SREVALDLG or raises the existing
%      singleton*.
%
%      H = SREVALDLG returns the handle to a new SREVALDLG or the handle to
%      the existing singleton*.
%
%      SREVALDLG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SREVALDLG.M with the given input arguments.
%
%      SREVALDLG('Property','Value',...) creates a new SREVALDLG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before srevaldlg_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to srevaldlg_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help srevaldlg

% Last Modified by GUIDE v2.5 26-Nov-2020 21:20:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @srevaldlg_OpeningFcn, ...
                   'gui_OutputFcn',  @srevaldlg_OutputFcn, ...
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


% --- Executes just before srevaldlg is made visible.
function srevaldlg_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to srevaldlg (see VARARGIN)

% Choose default command line output for srevaldlg
handles.output = hObject;
movegui(gcf,'center');
% Update handles structure
guidata(hObject, handles);
init_gui(hObject, handles);

% UIWAIT makes srevaldlg wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function init_gui(hObject, handles)
glvs
if ~isfield(handles,'setting')
    % ampmin ampmax  t1min t2max t2/t1 amp t1 t2
    handles.setting = [10; 1; 0.05; 0.01; 1; 0.5; 10; 60; 0; 1000];
end
setting = handles.setting;
set(handles.eb, 'String', setting(1));      set(handles.phiz, 'String', setting(6));
set(handles.db, 'String', setting(2));      set(handles.vel, 'String', setting(7));
set(handles.phixy, 'String', setting(3));   set(handles.T, 'String', setting(8));
set(handles.dvel, 'String', setting(4));    set(handles.rollrate, 'String', setting(9));
set(handles.dpos, 'String', setting(5));    set(handles.dkg, 'String', setting(10));
guidata(hObject, handles);
axes(handles.axes1); cla;

function setting_change(handles, hObject, value, idx)
    if isnan(value), uiwait(msgbox('请输入一数值！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    if value<0, uiwait(msgbox('输入数值不能为负！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    if value>1200 && idx==8, uiwait(msgbox('总飞行时间须小于1200s！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    handles.setting(idx) = value;
    guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = srevaldlg_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function eb_Callback(hObject, eventdata, handles)
% hObject    handle to eb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of eb as text
%        str2double(get(hObject,'String')) returns contents of eb as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 1);


% --- Executes during object creation, after setting all properties.
function eb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function db_Callback(hObject, eventdata, handles)
% hObject    handle to db (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of db as text
%        str2double(get(hObject,'String')) returns contents of db as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 2);


% --- Executes during object creation, after setting all properties.
function db_CreateFcn(hObject, eventdata, handles)
% hObject    handle to db (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function phixy_Callback(hObject, eventdata, handles)
% hObject    handle to phixy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    setting_change(handles, hObject, str2double(get(hObject,'String')), 3);


% --- Executes during object creation, after setting all properties.
function phixy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phixy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function dvel_Callback(hObject, eventdata, handles)
% hObject    handle to dvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dvel as text
%        str2double(get(hObject,'String')) returns contents of dvel as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 4);


% --- Executes during object creation, after setting all properties.
function dvel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function dpos_Callback(hObject, eventdata, handles)
% hObject    handle to dpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dpos as text
%        str2double(get(hObject,'String')) returns contents of dpos as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 5);


% --- Executes during object creation, after setting all properties.
function dpos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function phiz_Callback(hObject, eventdata, handles)
% hObject    handle to phiz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiz as text
%        str2double(get(hObject,'String')) returns contents of phiz as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 6);


% --- Executes during object creation, after setting all properties.
function phiz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function vel_Callback(hObject, eventdata, handles)
% hObject    handle to vel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vel as text
%        str2double(get(hObject,'String')) returns contents of vel as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 7);
    

% --- Executes during object creation, after setting all properties.
function vel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function T_Callback(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of T as text
%        str2double(get(hObject,'String')) returns contents of T as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 8);


% --- Executes during object creation, after setting all properties.
function T_CreateFcn(hObject, eventdata, handles)
% hObject    handle to T (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function rollrate_Callback(hObject, eventdata, handles)
% hObject    handle to rollrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rollrate as text
%        str2double(get(hObject,'String')) returns contents of rollrate as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 9);


% --- Executes during object creation, after setting all properties.
function rollrate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rollrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function dkg_Callback(hObject, eventdata, handles)
% hObject    handle to dkg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dkg as text
%        str2double(get(hObject,'String')) returns contents of dkg as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 10);


% --- Executes during object creation, after setting all properties.
function dkg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dkg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in savesetting.
function savesetting_Callback(hObject, eventdata, handles)
% hObject    handle to savesetting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uiputfile( {'*.sre'}, '保存参数');
if isequal(filename,0), return; end
cd(pathname);
fid = fopen([pathname,filename],'w');
fprintf(fid, '%.6f \n', handles.setting');
fclose(fid);

% --- Executes on button press in loadsetting.
function loadsetting_Callback(hObject, eventdata, handles)
% hObject    handle to loadsetting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( {'*.sre'}, '读取参数');
if isequal(filename,0), return; end
cd(pathname);
handles.setting = load([pathname,filename]);
init_gui(hObject, handles);
guidata(hObject, handles);

% --- Executes on button press in simulate.
function simulate_Callback(hObject, eventdata, handles)
% hObject    handle to simulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setting = handles.setting;
err = sreval([setting(1),setting(2),setting(3),setting(4),setting(5)], ...
    setting(8), [setting(7),setting(6)], [setting(9)/360,setting(10)], 0);
axes(handles.axes1); cla;
bar(err);
xlabel('误差因素'); ylabel(sprintf('导航误差评估 / m'));
set(handles.axes1, 'YMinorGrid', 'on');
xtl = {'eb', 'db', 'phixy0', 'dvel0', 'dpos0', 'phiz0*dvel*T', 'wy*dkg', 'TotalError'};
set(handles.axes1, 'XTicklabel', xtl);
    
% set(handles.eb, 'String', setting(1));   set(handles.dvel, 'String', setting(6));
% set(handles.db, 'String', setting(2));   set(handles.vel, 'String', setting(7));
% set(handles.phixy, 'String', setting(3));   set(handles.T, 'String', setting(8));
% set(handles.phiz, 'String', setting(4));   set(handles.rollrate, 'String', setting(9));
% set(handles.dpos, 'String', setting(5));    set(handles.dkg, 'String', setting(10));
