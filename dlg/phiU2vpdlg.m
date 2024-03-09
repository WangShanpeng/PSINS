function varargout = phiU2vpdlg(varargin)
% PHIU2VPDLG MATLAB code for phiU2vpdlg.fig
%      PHIU2VPDLG, by itself, creates a new PHIU2VPDLG or raises the existing
%      singleton*.
%
%      H = PHIU2VPDLG returns the handle to a new PHIU2VPDLG or the handle to
%      the existing singleton*.
%
%      PHIU2VPDLG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PHIU2VPDLG.M with the given input arguments.
%
%      PHIU2VPDLG('Property','Value',...) creates a new PHIU2VPDLG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before phiU2vpdlg_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to phiU2vpdlg_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help phiU2vpdlg

% Last Modified by GUIDE v2.5 28-Oct-2023 11:27:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @phiU2vpdlg_OpeningFcn, ...
                   'gui_OutputFcn',  @phiU2vpdlg_OutputFcn, ...
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


% --- Executes just before phiU2vpdlg is made visible.
function phiU2vpdlg_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to phiU2vpdlg (see VARARGIN)

% Choose default command line output for phiU2vpdlg
handles.output = hObject;
movegui(gcf,'center');
% Update handles structure
guidata(hObject, handles);
init_gui(hObject, handles);

% UIWAIT makes phiU2vpdlg wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function init_gui(hObject, handles)
glvs
if ~isfield(handles,'setting')
    handles.setting = [34; 1; 300];
end
setting = handles.setting;
set(handles.L0, 'String', setting(1));      
set(handles.phiU, 'String', setting(2));
set(handles.T, 'String', setting(3));
guidata(hObject, handles);
axes(handles.axes1); cla;

function setting_change(handles, hObject, value, idx)
    if isnan(value), uiwait(msgbox('请输入一数值！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    if (value>80 || value<-80) && idx==1, uiwait(msgbox('纬度须在-80~+80°！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    if (value>600 || value<-600) && idx==2, uiwait(msgbox('方位误差须在-600~+600角分！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    if (value<10 || value>1800) && idx==3, uiwait(msgbox('总时间须在10~1800s！','modal')); set(hObject,'String',handles.setting(idx)); return; end
    handles.setting(idx) = value;
    guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = phiU2vpdlg_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function L0_Callback(hObject, eventdata, handles)
% hObject    handle to L0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L0 as text
%        str2double(get(hObject,'String')) returns contents of L0 as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 1);


% --- Executes during object creation, after setting all properties.
function L0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function phiU_Callback(hObject, eventdata, handles)
% hObject    handle to phiU (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of phiU as text
%        str2double(get(hObject,'String')) returns contents of phiU as a double
    setting_change(handles, hObject, str2double(get(hObject,'String')), 2);


% --- Executes during object creation, after setting all properties.
function phiU_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phiU (see GCBO)
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
    setting_change(handles, hObject, str2double(get(hObject,'String')), 3);


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


% --- Executes on button press in simulate.
function simulate_Callback(hObject, eventdata, handles)
% hObject    handle to simulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global glv
setting = handles.setting;
t = 0:setting(3);
dvN = -setting(2)*glv.min*glv.g0*glv.wie*cos(setting(1)*glv.deg)*t.^2/2;
dL = dvN.*t/3;
axes(handles.axes1); cla;
ax = plotyy(gca, t,dvN, t,dL);  grid on;
xlabel('t / s');
set(get(ax(1),'Ylabel'), 'String', '北速误差 / m/s');
set(get(ax(2),'Ylabel'), 'String', '纬度误差 / m');
