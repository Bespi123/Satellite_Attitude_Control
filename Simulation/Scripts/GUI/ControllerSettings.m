function varargout = ControllerSettings(varargin)
%LS2125204: Brayan Espinoza
% CONTROLLERSETTINGS MATLAB code for ControllerSettings.fig
%      CONTROLLERSETTINGS, by itself, creates a new CONTROLLERSETTINGS or raises the existing
%      singleton*.
%
%      H = CONTROLLERSETTINGS returns the handle to a new CONTROLLERSETTINGS or the handle to
%      the existing singleton*.
%
%      CONTROLLERSETTINGS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROLLERSETTINGS.M with the given input arguments.
%
%      CONTROLLERSETTINGS('Property','Value',...) creates a new CONTROLLERSETTINGS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ControllerSettings_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ControllerSettings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ControllerSettings

% Last Modified by GUIDE v2.5 03-Dec-2021 19:51:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ControllerSettings_OpeningFcn, ...
                   'gui_OutputFcn',  @ControllerSettings_OutputFcn, ...
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


% --- Executes just before ControllerSettings is made visible.
function ControllerSettings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ControllerSettings (see VARARGIN)

% Choose default command line output for ControllerSettings
handles.output = hObject;
if (isempty(varargin) == 0)
    %Desired Euler angles
    euler = quat2eul(varargin{1,1}')*180/pi;
    set(handles.roll_edit, 'String', num2str(euler(1),'%.2f'));
    set(handles.pitch_edit, 'String', num2str(euler(2),'%.2f'));
    set(handles.yaw_edit, 'String', num2str(euler(3),'%.2f'));
    %Controller parameters
    set(handles.K_edit, 'String', num2str(varargin{1,3}(1,1),'%.2f'));
    set(handles.P_edit, 'String', num2str(varargin{1,4}(1,1),'%.2f'));
end
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ControllerSettings wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ControllerSettings_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Applu_pushbutton.
function Applu_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to Applu_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% LS2125204 Read Set point
Roll = str2double(get(handles.roll_edit, 'string'));
Pitch = str2double(get(handles.pitch_edit, 'string'));
Yaw = str2double(get(handles.yaw_edit, 'string'));
Wx = str2double(get(handles.wx_edit, 'string'));
Wy = str2double(get(handles.wy_edit, 'string'));
Wz = str2double(get(handles.wz_edit, 'string'));
K = str2double(get(handles.K_edit, 'string'));
P = str2double(get(handles.P_edit, 'string'));
if isnan(Roll)
    errordlg('Invalid value of Roll.', 'Invalid Value', 'modal');
elseif isnan(Pitch)
    errordlg('Invalid value of Pitch.', 'Invalid Value', 'modal');    
elseif isnan(Yaw)
    errordlg('Invalid value of Yaw.', 'Invalid Value', 'modal');
elseif isnan(Wx)
    errordlg('Invalid value of Wx.', 'Invalid Value', 'modal');    
elseif isnan(Wy)
    errordlg('Invalid value of Wy.', 'Invalid Value', 'modal');
elseif isnan(Wz)
    errordlg('Invalid value of Wz.', 'Invalid Value', 'modal');  
elseif isnan(K)
    errordlg('Invalid value of K.', 'Invalid Value', 'modal');
elseif isnan(P)
    errordlg('Invalid value of P.', 'Invalid Value', 'modal');
else
    handles.qd=eul2quat(pi/180*[Roll,Pitch,Yaw])';
    handles.Wd=pi/180*[Wx,Wy,Wz]';  %rad/s
    handles.Keye=K*eye(3,3);  
    handles.Peye=P*eye(3,3);  
    LS2125204(2,handles.qd,handles.Wd,handles.Keye,handles.Peye);
end

%Update handles structure
guidata(hObject, handles)



function K_edit_Callback(hObject, eventdata, handles)
% hObject    handle to K_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of K_edit as text
%        str2double(get(hObject,'String')) returns contents of K_edit as a double


% --- Executes during object creation, after setting all properties.
function K_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to K_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function P_edit_Callback(hObject, eventdata, handles)
% hObject    handle to P_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of P_edit as text
%        str2double(get(hObject,'String')) returns contents of P_edit as a double


% --- Executes during object creation, after setting all properties.
function P_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_edit_Callback(hObject, eventdata, handles)
% hObject    handle to roll_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll_edit as text
%        str2double(get(hObject,'String')) returns contents of roll_edit as a double


% --- Executes during object creation, after setting all properties.
function roll_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pitch_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pitch_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pitch_edit as text
%        str2double(get(hObject,'String')) returns contents of pitch_edit as a double


% --- Executes during object creation, after setting all properties.
function pitch_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yaw_edit_Callback(hObject, eventdata, handles)
% hObject    handle to yaw_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yaw_edit as text
%        str2double(get(hObject,'String')) returns contents of yaw_edit as a double


% --- Executes during object creation, after setting all properties.
function yaw_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function wx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to wx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wx_edit as text
%        str2double(get(hObject,'String')) returns contents of wx_edit as a double


% --- Executes during object creation, after setting all properties.
function wx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function wy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to wy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wy_edit as text
%        str2double(get(hObject,'String')) returns contents of wy_edit as a double


% --- Executes during object creation, after setting all properties.
function wy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function wz_edit_Callback(hObject, eventdata, handles)
% hObject    handle to wz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wz_edit as text
%        str2double(get(hObject,'String')) returns contents of wz_edit as a double


% --- Executes during object creation, after setting all properties.
function wz_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
