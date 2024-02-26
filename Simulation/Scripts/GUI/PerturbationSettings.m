function varargout = PerturbationSettings(varargin)
%LS2125204: Brayan Espinoza
% PERTURBATIONSETTINGS MATLAB code for PerturbationSettings.fig
%      PERTURBATIONSETTINGS, by itself, creates a new PERTURBATIONSETTINGS or raises the existing
%      singleton*.
%
%      H = PERTURBATIONSETTINGS returns the handle to a new PERTURBATIONSETTINGS or the handle to
%      the existing singleton*.
%
%      PERTURBATIONSETTINGS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PERTURBATIONSETTINGS.M with the given input arguments.
%
%      PERTURBATIONSETTINGS('Property','Value',...) creates a new PERTURBATIONSETTINGS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PerturbationSettings_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PerturbationSettings_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PerturbationSettings

% Last Modified by GUIDE v2.5 04-Dec-2021 13:47:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PerturbationSettings_OpeningFcn, ...
                   'gui_OutputFcn',  @PerturbationSettings_OutputFcn, ...
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


% --- Executes just before PerturbationSettings is made visible.
function PerturbationSettings_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PerturbationSettings (see VARARGIN)

% Choose default command line output for PerturbationSettings
handles.output = hObject;
if (isempty(varargin) == 0)
    % Constant perturbation
    set(handles.Tgx_edit, 'String', num2str(varargin{1,1},'%.2f'));
    set(handles.Tgy_edit, 'String', num2str(varargin{1,2},'%.2f'));
    set(handles.Tgz_edit, 'String', num2str(varargin{1,3},'%.2f'));
    % Miscelaneous perturbation
    set(handles.A_edit, 'String', num2str(varargin{1,4},'%.2f'));
    set(handles.w1_edit, 'String', num2str(varargin{1,5},'%.2f'));
    set(handles.w2_edit, 'String', num2str(varargin{1,6},'%.2f'));
    set(handles.w3_edit, 'String', num2str(varargin{1,7},'%.2f'));
end
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PerturbationSettings wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PerturbationSettings_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function w1_edit_Callback(hObject, eventdata, handles)
% hObject    handle to w1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of w1_edit as text
%        str2double(get(hObject,'String')) returns contents of w1_edit as a double


% --- Executes during object creation, after setting all properties.
function w1_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to w1_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function w2_edit_Callback(hObject, eventdata, handles)
% hObject    handle to w2_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of w2_edit as text
%        str2double(get(hObject,'String')) returns contents of w2_edit as a double


% --- Executes during object creation, after setting all properties.
function w2_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to w2_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function w3_edit_Callback(hObject, eventdata, handles)
% hObject    handle to w3_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of w3_edit as text
%        str2double(get(hObject,'String')) returns contents of w3_edit as a double


% --- Executes during object creation, after setting all properties.
function w3_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to w3_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function A_edit_Callback(hObject, eventdata, handles)
% hObject    handle to A_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A_edit as text
%        str2double(get(hObject,'String')) returns contents of A_edit as a double


% --- Executes during object creation, after setting all properties.
function A_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in miscPer_pushbutton.
function miscPer_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to miscPer_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
A = str2double(get(handles.A_edit, 'string'));
w1 = str2double(get(handles.w1_edit, 'string'));
w2 = str2double(get(handles.w2_edit, 'string'));
w3 = str2double(get(handles.w3_edit, 'string'));

if isnan(A)
    errordlg('Invalid value of A.', 'Invalid Value', 'modal');
elseif isnan(w1)
    errordlg('Invalid value of w1.', 'Invalid Value', 'modal');    
elseif isnan(w2)
    errordlg('Invalid value of w2.', 'Invalid Value', 'modal');
elseif isnan(w3)
    errordlg('Invalid value of w3.', 'Invalid Value', 'modal');
else
    %handles.Tg=A*[sin(w1*handles.t);sin(w2*handles.t);sin(w3*handles.t)];
    LS2125204(4,A,w1,w2,w3);
end

%Update handles structure
guidata(hObject, handles)



function Tgx_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Tgx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tgx_edit as text
%        str2double(get(hObject,'String')) returns contents of Tgx_edit as a double


% --- Executes during object creation, after setting all properties.
function Tgx_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tgx_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tgy_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Tgy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tgy_edit as text
%        str2double(get(hObject,'String')) returns contents of Tgy_edit as a double


% --- Executes during object creation, after setting all properties.
function Tgy_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tgy_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tgz_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Tgz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tgz_edit as text
%        str2double(get(hObject,'String')) returns contents of Tgz_edit as a double


% --- Executes during object creation, after setting all properties.
function Tgz_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tgz_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in constPer_pushbutton.
function constPer_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to constPer_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Tgx = str2double(get(handles.Tgx_edit, 'string'));
Tgy = str2double(get(handles.Tgy_edit, 'string'));
Tgz = str2double(get(handles.Tgz_edit, 'string'));

if isnan(Tgx)
    errordlg('Invalid value of Tgx.', 'Invalid Value', 'modal');
elseif isnan(Tgy)
    errordlg('Invalid value of Tgy.', 'Invalid Value', 'modal');    
elseif isnan(Tgz)
    errordlg('Invalid value of Tgz.', 'Invalid Value', 'modal');
else
    %handles.Tg=[Tgx*(ones(handles.n,1)),Tgy*(ones(handles.n,1)),...
    %      Tgz*(ones(handles.n,1))]';
    LS2125204(3,Tgx,Tgy,Tgz);
end

%Update handles structure
guidata(hObject, handles)
