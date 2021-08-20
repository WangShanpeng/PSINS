function varargout = seawavedlg(varargin)
% SEAWAVEDLG MATLAB code for seawavedlg.fig
%      SEAWAVEDLG, by itself, creates a new SEAWAVEDLG or raises the existing
%      singleton*.
%
%      H = SEAWAVEDLG returns the handle to a new SEAWAVEDLG or the handle to
%      the existing singleton*.
%
%      SEAWAVEDLG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SEAWAVEDLG.M with the given input arguments.
%
%      SEAWAVEDLG('Property','Value',...) creates a new SEAWAVEDLG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before seawavedlg_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to seawavedlg_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help seawavedlg

% Last Modified by GUIDE v2.5 07-Jul-2019 11:31:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @seawavedlg_OpeningFcn, ...
                   'gui_OutputFcn',  @seawavedlg_OutputFcn, ...
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


% --- Executes just before seawavedlg is made visible.
function seawavedlg_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to seawavedlg (see VARARGIN)

% Choose default command line output for seawavedlg
handles.output = hObject;
movegui(gcf,'center');
% Update handles structure
guidata(hObject, handles);
init_gui(hObject, handles);

% UIWAIT makes seawavedlg wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function init_gui(hObject, handles)
if ~isfield(handles,'setting')
    % ampmin ampmax  t1min t2max t2/t1 amp t1 t2
    handles.setting = [[0.001 45 1 30 1.5 4 4 8];
        [0.001 45 2 30 1.5 5 6 10];
        [0.001 45 2 30 1.5 6 5 8];
        [0.001 10 2 30 1.5 0.5 3 8];
        [0.001 10 2 30 1.5 0.8 5 9];
        [0.001 10 2 30 1.5 1.0 8 12]];
    handles.T = 60;
end
setting = handles.setting;
set(handles.pitchamp, 'String', setting(1,6)); set(handles.pitcht1, 'String', setting(1,7)); set(handles.pitcht2, 'String', setting(1,8)); 
set(handles.rollamp, 'String', setting(2,6)); set(handles.rollt1, 'String', setting(2,7)); set(handles.rollt2, 'String', setting(2,8)); 
set(handles.yawamp, 'String', setting(3,6)); set(handles.yawt1, 'String', setting(3,7)); set(handles.yawt2, 'String', setting(3,8)); 
set(handles.swayamp, 'String', setting(4,6)); set(handles.swayt1, 'String', setting(4,7)); set(handles.swayt2, 'String', setting(4,8)); 
set(handles.surgeamp, 'String', setting(5,6)); set(handles.surget1, 'String', setting(5,7)); set(handles.surget2, 'String', setting(5,8)); 
set(handles.heaveamp, 'String', setting(6,6)); set(handles.heavet1, 'String', setting(6,7)); set(handles.heavet2, 'String', setting(6,8)); 
set(handles.simut, 'String', handles.T);
guidata(hObject, handles);
axes(handles.axes1); cla; grid on; ylabel('角位移幅值 / (\circ)');
axes(handles.axes2); cla; grid on; xlabel('t / s'); ylabel('线位移幅值 / m');

function setting_change(handles, hObject, value, idx, clm)
    if isnan(value), uiwait(msgbox('请输入一数值！','modal')); set(hObject,'String',handles.setting(idx,clm)); return; end
    if clm==6, minval=handles.setting(idx,1); maxval=handles.setting(idx,2); end
    if clm==7, minval=handles.setting(idx,3); maxval=handles.setting(idx,4); end
    if clm==8, minval=handles.setting(idx,3); maxval=handles.setting(idx,4); end
    if value<minval||value>maxval, uiwait(msgbox(sprintf('输入数值需在%.3f~%.3f之间！',minval,maxval),'modal')); set(hObject,'String',handles.setting(idx,clm)); return;  end
    if clm==7, 
        T1max = handles.setting(idx,8)/handles.setting(idx,5);
        if value>T1max, uiwait(msgbox(sprintf('输入数值需小于%.3f（T2/%.3f）！',T1max,handles.setting(idx,5)),'modal')); set(hObject,'String',handles.setting(idx,clm)); return;  end
    end
    if clm==8,
        T2min = handles.setting(idx,7)*handles.setting(idx,5);
        if value<T2min, uiwait(msgbox(sprintf('输入数值需大于%.3f（T1*%.3f）！',T2min,handles.setting(idx,5)),'modal')); set(hObject,'String',handles.setting(idx,clm)); return;  end
    end
    handles.setting(idx,clm) = value;
    guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = seawavedlg_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function pitchamp_Callback(hObject, eventdata, handles)
% hObject    handle to pitchamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitchamp as text
%        str2double(get(hObject,'String')) returns contents of pitchamp as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 1, 6);


% --- Executes during object creation, after setting all properties.
function pitchamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitchamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rollamp_Callback(hObject, eventdata, handles)
% hObject    handle to rollamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rollamp as text
%        str2double(get(hObject,'String')) returns contents of rollamp as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 2, 6);


% --- Executes during object creation, after setting all properties.
function rollamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rollamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function yawamp_Callback(hObject, eventdata, handles)
% hObject    handle to yawamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    setting_change(handles, hObject, str2double(get(hObject,'String')), 3, 6);


% --- Executes during object creation, after setting all properties.
function yawamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yawamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function swayamp_Callback(hObject, eventdata, handles)
% hObject    handle to swayamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of swayamp as text
%        str2double(get(hObject,'String')) returns contents of swayamp as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 4, 6);


% --- Executes during object creation, after setting all properties.
function swayamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to swayamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function surgeamp_Callback(hObject, eventdata, handles)
% hObject    handle to surgeamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of surgeamp as text
%        str2double(get(hObject,'String')) returns contents of surgeamp as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 5, 6);


% --- Executes during object creation, after setting all properties.
function surgeamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to surgeamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function heaveamp_Callback(hObject, eventdata, handles)
% hObject    handle to heaveamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of heaveamp as text
%        str2double(get(hObject,'String')) returns contents of heaveamp as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 6, 6);


% --- Executes during object creation, after setting all properties.
function heaveamp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heaveamp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitcht1_Callback(hObject, eventdata, handles)
% hObject    handle to pitcht1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitcht1 as text
%        str2double(get(hObject,'String')) returns contents of pitcht1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 1, 7);
    

% --- Executes during object creation, after setting all properties.
function pitcht1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitcht1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rollt1_Callback(hObject, eventdata, handles)
% hObject    handle to rollt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rollt1 as text
%        str2double(get(hObject,'String')) returns contents of rollt1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 2, 7);


% --- Executes during object creation, after setting all properties.
function rollt1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rollt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function yawt1_Callback(hObject, eventdata, handles)
% hObject    handle to yawt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yawt1 as text
%        str2double(get(hObject,'String')) returns contents of yawt1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 3, 7);


% --- Executes during object creation, after setting all properties.
function yawt1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yawt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function swayt1_Callback(hObject, eventdata, handles)
% hObject    handle to swayt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of swayt1 as text
%        str2double(get(hObject,'String')) returns contents of swayt1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 4, 7);


% --- Executes during object creation, after setting all properties.
function swayt1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to swayt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function surget1_Callback(hObject, eventdata, handles)
% hObject    handle to surget1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of surget1 as text
%        str2double(get(hObject,'String')) returns contents of surget1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 5, 7);


% --- Executes during object creation, after setting all properties.
function surget1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to surget1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function heavet1_Callback(hObject, eventdata, handles)
% hObject    handle to heavet1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of heavet1 as text
%        str2double(get(hObject,'String')) returns contents of heavet1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 6, 7);


% --- Executes during object creation, after setting all properties.
function heavet1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heavet1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitcht2_Callback(hObject, eventdata, handles)
% hObject    handle to pitcht2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitcht2 as text
%        str2double(get(hObject,'String')) returns contents of pitcht2 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 1, 8);
    

% --- Executes during object creation, after setting all properties.
function pitcht2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitcht2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rollt2_Callback(hObject, eventdata, handles)
% hObject    handle to rollt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rollt2 as text
%        str2double(get(hObject,'String')) returns contents of rollt2 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 2, 8);


% --- Executes during object creation, after setting all properties.
function rollt2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rollt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function yawt2_Callback(hObject, eventdata, handles)
% hObject    handle to yawt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yawt1 as text
%        str2double(get(hObject,'String')) returns contents of yawt1 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 3, 7);


% --- Executes during object creation, after setting all properties.
function yawt2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yawt1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function swayt2_Callback(hObject, eventdata, handles)
% hObject    handle to swayt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of swayt2 as text
%        str2double(get(hObject,'String')) returns contents of swayt2 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 4, 8);


% --- Executes during object creation, after setting all properties.
function swayt2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to swayt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function surget2_Callback(hObject, eventdata, handles)
% hObject    handle to surget2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of surget2 as text
%        str2double(get(hObject,'String')) returns contents of surget2 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 5, 8);


% --- Executes during object creation, after setting all properties.
function surget2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to surget2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function heavet2_Callback(hObject, eventdata, handles)
% hObject    handle to heavet2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of heavet2 as text
%        str2double(get(hObject,'String')) returns contents of heavet2 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 6, 8);


% --- Executes during object creation, after setting all properties.
function heavet2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to heavet2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function simut_Callback(hObject, eventdata, handles)
% hObject    handle to simut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simut as text
%        str2double(get(hObject,'String')) returns contents of simut as a double
value = str2double(get(hObject,'String'));
if isnan(value), uiwait(msgbox('请输入一数值！','modal')); set(hObject,'String',handles.simut); return; end
if value<30||value>1800, uiwait(msgbox(sprintf('输入数值需在%.3f~%.3fs之间！',30,1800),'modal')); set(hObject,'String',handles.T); return;  end
handles.T = value;
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function simut_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simut (see GCBO)
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
[filename, pathname] = uiputfile( {'*.shp'}, '保存参数');
if isequal(filename,0), return; end
cd(pathname);
fid = fopen([pathname,filename],'w');
fprintf(fid, '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n', handles.setting');
fclose(fid);

% --- Executes on button press in loadsetting.
function loadsetting_Callback(hObject, eventdata, handles)
% hObject    handle to loadsetting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( {'*.shp'}, '读取参数');
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
apt = seawave6(handles.setting(:,end-2:end), handles.T-13, 0.01, 3, 10);
axes(handles.axes1); cla;
plot(apt(:,end),apt(:,1:3)); grid on; ylabel('角位移幅值 / (\circ)'); legend('纵摇','横摇','艏摇');
axes(handles.axes2); cla;
plot(apt(:,end),apt(:,4:6)); grid on; xlabel('t / s'); ylabel('线位移幅值 / m'); legend('横荡','纵荡','垂荡');
handles.apt = apt;
guidata(hObject, handles);

% --- Executes on button press in savedata.
function savedata_Callback(hObject, eventdata, handles)
% hObject    handle to savedata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isfield(handles,'apt'), msgbox('请先仿真生成数据！', 'modal'); return; end;
[filename, pathname] = uiputfile( {'*.txt';'*.dat'}, '保存数据');
if isequal(filename,0), return; end
cd(pathname);
fid = fopen([pathname,filename],'w');
fprintf(fid, '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n', [handles.apt(:,end),handles.apt(:,4:6)*1000,handles.apt(:,1:3)]');
fclose(fid);
