function varargout = CubeSarSettings(varargin)
% LS2125204: Brayan Espinoza
% CUBESARSETTINGS MATLAB code for CubeSarSettings.fig
%      CUBESARSETTINGS, by itself, creates a new CUBESARSETTINGS or raises the existing
%      singleton*.
%
%      H = CUBESARSETTINGS returns the handle to a new CUBESARSETTINGS or the handle to
%      the existing singleton*.
%
%      CUBESARSETTINGS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CUBESARSETTINGS.M with the given input arguments.
%
%      CUBESARSETTINGS('Property','Value',...) creates a new CUBESARSETTINGS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CubeSarSettings_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CubeSarSettings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CubeSarSettings

% Last Modified by GUIDE v2.5 03-Dec-2021 14:24:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CubeSarSettings_OpeningFcn, ...
                   'gui_OutputFcn',  @CubeSarSettings_OutputFcn, ...
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


% --- Executes just before CubeSarSettings is made visible.
function CubeSarSettings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CubeSarSettings (see VARARGIN)

% Choose default command line output for CubeSarSettings
handles.output = hObject;
if (isempty(varargin) == 0)
    % Write Inertia tensor in GUIDE
    set(handles.Ixx_edit, 'String', num2str(varargin{1,1}(1,1),'%.2f'));
    set(handles.Iyx_edit, 'String', num2str(varargin{1,1}(1,2),'%.2f'));
    set(handles.Izx_edit, 'String', num2str(varargin{1,1}(1,3),'%.2f'));
    set(handles.Ixy_edit, 'String', num2str(varargin{1,1}(2,1),'%.2f'));
    set(handles.Iyy_edit, 'String', num2str(varargin{1,1}(2,2),'%.2f'));
    set(handles.Izy_edit, 'String', num2str(varargin{1,1}(2,3),'%.2f'));
    set(handles.Ixz_edit, 'String', num2str(varargin{1,1}(3,1),'%.2f'));
    set(handles.Iyz_edit, 'String', num2str(varargin{1,1}(3,2),'%.2f'));
    set(handles.Izz_edit, 'String', num2str(varargin{1,1}(3,3),'%.2f'));
    %Initial Euler angles
    euler = quat2eul(varargin{1,2}')*180/pi;
    set(handles.roll_edit, 'String', num2str(euler(1),'%.2f'));
    set(handles.pitch_edit, 'String', num2str(euler(2),'%.2f'));
    set(handles.yaw_edit, 'String', num2str(euler(3),'%.2f'));
    %Initial Angular Rates
    rate = varargin{1,3}*180/pi;
    set(handles.ratex_edit, 'String', num2str(rate(1),'%.2f'));
    set(handles.ratey_edit, 'String', num2str(rate(2),'%.2f'));
    set(handles.ratez_edit, 'String', num2str(rate(3),'%.2f'));
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CubeSarSettings wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CubeSarSettings_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Ixx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Ixx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ixx_edit as text
%        str2double(get(hObject,'String')) returns contents of Ixx_edit as a double


% --- Executes during object creation, after setting all properties.
function Ixx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ixx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iyx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Iyx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iyx_edit as text
%        str2double(get(hObject,'String')) returns contents of Iyx_edit as a double


% --- Executes during object creation, after setting all properties.
function Iyx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iyx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Izx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Izx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Izx_edit as text
%        str2double(get(hObject,'String')) returns contents of Izx_edit as a double


% --- Executes during object creation, after setting all properties.
function Izx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Izx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ixy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Ixy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ixy_edit as text
%        str2double(get(hObject,'String')) returns contents of Ixy_edit as a double


% --- Executes during object creation, after setting all properties.
function Ixy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ixy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ixz_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Ixz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ixz_edit as text
%        str2double(get(hObject,'String')) returns contents of Ixz_edit as a double


% --- Executes during object creation, after setting all properties.
function Ixz_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ixz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iyy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Iyy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iyy_edit as text
%        str2double(get(hObject,'String')) returns contents of Iyy_edit as a double


% --- Executes during object creation, after setting all properties.
function Iyy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iyy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iyz_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Iyz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iyz_edit as text
%        str2double(get(hObject,'String')) returns contents of Iyz_edit as a double


% --- Executes during object creation, after setting all properties.
function Iyz_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Iyz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Izy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Izy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Izy_edit as text
%        str2double(get(hObject,'String')) returns contents of Izy_edit as a double


% --- Executes during object creation, after setting all properties.
function Izy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Izy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Izz_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Izz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Izz_edit as text
%        str2double(get(hObject,'String')) returns contents of Izz_edit as a double


% --- Executes during object creation, after setting all properties.
function Izz_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Izz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ok_button.
function ok_button_Callback(hObject, eventdata, handles)
% hObject    handle to ok_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% LS2125204: Read Inertia tensor values
Ixx = str2double(get(handles.Ixx_edit, 'string'));
Ixy = str2double(get(handles.Ixy_edit, 'string'));
Ixz = str2double(get(handles.Ixz_edit, 'string'));
Iyx = str2double(get(handles.Iyx_edit, 'string'));
Iyy = str2double(get(handles.Iyy_edit, 'string'));
Iyz = str2double(get(handles.Iyz_edit, 'string'));
Izx = str2double(get(handles.Izx_edit, 'string'));
Izy = str2double(get(handles.Izy_edit, 'string'));
Izz = str2double(get(handles.Izz_edit, 'string'));

% LS2125204 Read Initial conditions
Roll = str2double(get(handles.roll_edit, 'string'));
Pitch = str2double(get(handles.pitch_edit, 'string'));
Yaw = str2double(get(handles.yaw_edit, 'string'));
Wx = str2double(get(handles.ratex_edit, 'string'));
Wy = str2double(get(handles.ratey_edit, 'string'));
Wz = str2double(get(handles.ratez_edit, 'string'));

if isnan(Ixx)
    errordlg('Invalid value of Ixx.', 'Invalid Value', 'modal');
elseif isnan(Ixy)
    errordlg('Invalid value of Ixy.', 'Invalid Value', 'modal');    
elseif isnan(Ixz)
    errordlg('Invalid value of Ixz.', 'Invalid Value', 'modal');
elseif isnan(Iyx)
    errordlg('Invalid value of Iyx.', 'Invalid Value', 'modal');    
elseif isnan(Iyy)
    errordlg('Invalid value of Iyy.', 'Invalid Value', 'modal');
elseif isnan(Iyz)
    errordlg('Invalid value of Iyz.', 'Invalid Value', 'modal');    
elseif isnan(Izx)
    errordlg('Invalid value of Izx.', 'Invalid Value', 'modal');
elseif isnan(Izy)
    errordlg('Invalid value of Izy.', 'Invalid Value', 'modal');    
elseif isnan(Izz)
    errordlg('Invalid value of Izz.', 'Invalid Value', 'modal');
elseif isnan(Roll)
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
else
    handles.I=[Ixx,Iyx,Izx;
       Ixy,Iyy,Izy;
       Ixz,Iyz,Izz];
    handles.q0=eul2quat(pi/180*[Roll,Pitch,Yaw])';
    handles.W0=pi/180*[Wx,Wy,Wz]';  %rad/s
    
    LS2125204(1,handles.I,handles.q0,handles.W0);   
end

%Update handles structure
guidata(hObject, handles)



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



function ratex_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ratex_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ratex_edit as text
%        str2double(get(hObject,'String')) returns contents of ratex_edit as a double


% --- Executes during object creation, after setting all properties.
function ratex_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ratex_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ratey_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ratey_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ratey_edit as text
%        str2double(get(hObject,'String')) returns contents of ratey_edit as a double


% --- Executes during object creation, after setting all properties.
function ratey_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ratey_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ratez_edit_Callback(hObject, eventdata, handles)
% hObject    handle to ratez_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ratez_edit as text
%        str2double(get(hObject,'String')) returns contents of ratez_edit as a double


% --- Executes during object creation, after setting all properties.
function ratez_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ratez_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function ok_button_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ok_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
