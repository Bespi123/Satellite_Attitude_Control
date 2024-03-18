function varargout = LS2125204(varargin)
% LS2125204 MATLAB code for LS2125204.fig
%      LS2125204, by itself, creates a new LS2125204 or raises the existing
%      singleton*.
%
%      H = LS2125204 returns the handle to a new LS2125204 or the handle to
%      the existing singleton*.
%
%      LS2125204('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LS2125204.M with the given input arguments.
%
%      LS2125204('Property','Value',...) creates a new LS2125204 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LS2125204_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LS2125204_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LS2125204

% Last Modified by GUIDE v2.5 03-Dec-2021 14:05:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LS2125204_OpeningFcn, ...
                   'gui_OutputFcn',  @LS2125204_OutputFcn, ...
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


% --- Executes just before LS2125204 is made visible.
function LS2125204_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LS2125204 (see VARARGIN)

% Choose default command line output for LS2125204
handles.output = hObject;

if (isempty(varargin) == 0)
    n = varargin{1};
    switch n
    case 1
        handles.I  = varargin{2};
        handles.q0 = varargin{3};
        handles.Wo = varargin{4};
    case 2
        handles.qd   = varargin{2};
        handles.Wd   = varargin{3}; %rad/s
        handles.Keye = varargin{4};
        handles.Peye = varargin{5};
    case 3
        handles.Tdx = varargin{2};
        handles.Tdy = varargin{3};
        handles.Tdz = varargin{4};
        handles.i=1;
    case 4
        handles.A = varargin{2};
        handles.w1 = varargin{3};
        handles.w2 = varargin{4};
        handles.w3 = varargin{5};
        handles.i=2;
    case 5
        handles.qd   = varargin{2};
        handles.Wd   = varargin{3};
        handles.delta = varargin{4};
        handles.gamma = varargin{5};
        handles.Umax  = varargin{6};
        handles.k0    = varargin{7};
   otherwise
        disp('other value')
    end
else
    % Initial default values
    %CubeSat Parameters
    handles.I =eye(3);
    handles.q0 =[1,0,0,0]';
    handles.Wo =[0,0,0]';
    %Controller Parameters
    handles.qd =[0.9808,0.1691,0.0933,0.0277]';
    handles.Wd =[0,0,0]'; %rad/s
    %Feedback Controller parameters
    handles.Keye = eye(3);
    handles.Peye = eye(3);
    %Boskovic Controller parameters
    handles.delta = 0.01;
    handles.gamma = 0.001;
    handles.Umax  = 0.10;
    handles.k0    = 1;
    %Disturbances torque parameters
    handles.Tdx = 0.001;
    handles.Tdy = 0.001;
    handles.Tdz = 0.001;
    handles.A = 0.01;
    handles.w1 = 1.00;
    handles.w2 = 2.00;
    handles.w3 = 3.00;
    handles.i=0;
end
        
% LS2125204: plot beihang logo 
axes(handles.logo);
%matlabImage = imread('logo.png');
matlabImage = imread('adcs.jpg');
image(matlabImage)
axis off
axis image

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LS2125204 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LS2125204_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in graphic1_popupmenu.
function graphic1_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to graphic1_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns graphic1_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from graphic1_popupmenu


% --- Executes during object creation, after setting all properties.
function graphic1_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to graphic1_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3


% --- Executes on button press in graphic1_button.
function graphic1_button_Callback(hObject, eventdata, handles)
% hObject    handle to graphic1_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
v = get(handles.graphic1_popupmenu, 'value');
switch v
    case 1
        axes(handles.graphic1_axes);
        plot(handles.t(1:handles.n-1),180/pi*handles.angArray)
        grid on; xlabel('Time(s)');
        ylabel('Euler Angles(deg)');
        legend('Roll','Pitch','Yaw');
    case 2
        axes(handles.graphic1_axes);    
        plot(handles.t(1:handles.n-1),handles.xArray(1:4,:))
        grid on; xlabel('Time(s)');
        legend('q0','q1','q2','q3');
        ylabel('Quaternions');
    case 3
        axes(handles.graphic1_axes);    
        plot(handles.t(1:handles.n-1),handles.xArray(5:7,:))
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Angular Rates Error(rad/s)');
    case 4
        axes(handles.graphic1_axes);    
        plot(handles.t,handles.Td)
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Disturbances Torques(Nm)');
    case 5
        axes(handles.graphic1_axes);    
        plot(handles.t(1:handles.n-1),handles.uiArray)
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Control Torque(Nm)'); 
    case 6
        axes(handles.graphic1_axes);    
        plot(handles.t(1:handles.n-1),handles.EULERINT)
        grid on; xlabel('Time(s)'); 
        ylabel('EULERINT (rad.s)');
    case 7
        axes(handles.graphic1_axes);    
        plot(handles.t(1:handles.n-1),handles.ASCCT)
        grid on; xlabel('Time(s)');
        ylabel('ASCCT (Nm)');
    case 8
        if handles.c1 == 2
            axes(handles.graphic1_axes);    
            plot(handles.t(1:handles.n-1),handles.kArray)
            grid on; xlabel('Time(s)');
            ylabel('Adaptive gain K');  
        else
            warndlg('Feedback Controller does not have K value.','K value', 'modal');
        end
    otherwise
        disp('other value')
end
%Update handles structure
guidata(hObject, handles)



% --- Executes on button press in graphic2_button.
function graphic2_button_Callback(hObject, eventdata, handles)
% hObject    handle to graphic2_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
v = get(handles.graphic2_popupmenu, 'value');
switch v
    case 1
        axes(handles.graphic2_axes);
        plot(handles.t(1:handles.n-1),180/pi*handles.angArray)
        grid on; xlabel('Time(s)');
        ylabel('Euler Angles(deg)');
        legend('Roll','Pitch','Yaw');
    case 2
        axes(handles.graphic2_axes);    
        plot(handles.t(1:handles.n-1),handles.xArray(1:4,:))
        grid on; xlabel('Time(s)');
        legend('q0','q1','q2','q3');
        ylabel('Quaternions');
    case 3
        axes(handles.graphic2_axes);    
        plot(handles.t(1:handles.n-1),handles.xArray(5:7,:))
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Angular Rates Error(rad/s)');
    case 4
        axes(handles.graphic2_axes);    
        plot(handles.t,handles.Td)
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Disturbances Torques(Nm)');
    case 5
        axes(handles.graphic2_axes);    
        plot(handles.t(1:handles.n-1),handles.uiArray)
        grid on; xlabel('Time(s)');
        legend('X','Y','Z');
        ylabel('Control Torque(Nm)'); 
    case 6
        axes(handles.graphic2_axes);    
        plot(handles.t(1:handles.n-1),handles.EULERINT)
        grid on; xlabel('Time(s)'); 
        ylabel('EULERINT (rad.s)');
    case 7
        axes(handles.graphic2_axes);    
        plot(handles.t(1:handles.n-1),handles.ASCCT)
        grid on; xlabel('Time(s)');
        ylabel('ASCCT (Nm)');
    case 8
        if handles.c1 == 2
            axes(handles.graphic2_axes);    
            plot(handles.t(1:handles.n-1),handles.kArray)
            grid on; xlabel('Time(s)');
            ylabel('Adaptive gain K');  
        else
            warndlg('Feedback Controller does not have K value.','K value', 'modal');
        end
    otherwise
        disp('other value')
end
%Update handles structure
guidata(hObject, handles)


% --- Executes on selection change in graphic2_popupmenu.
function graphic2_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to graphic2_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns graphic2_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from graphic2_popupmenu


% --- Executes during object creation, after setting all properties.
function graphic2_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to graphic2_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initTime_edit_Callback(hObject, eventdata, handles)
% hObject    handle to initTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initTime_edit as text
%        str2double(get(hObject,'String')) returns contents of initTime_edit as a double


% --- Executes during object creation, after setting all properties.
function initTime_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function finalTime_edit_Callback(hObject, eventdata, handles)
% hObject    handle to finalTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of finalTime_edit as text
%        str2double(get(hObject,'String')) returns contents of finalTime_edit as a double


% --- Executes during object creation, after setting all properties.
function finalTime_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to finalTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function step_edit_Callback(hObject, eventdata, handles)
% hObject    handle to step_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of step_edit as text
%        str2double(get(hObject,'String')) returns contents of step_edit as a double


% --- Executes during object creation, after setting all properties.
function step_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to step_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cubesat_button.
function cubesat_button_Callback(hObject, eventdata, handles)
% hObject    handle to cubesat_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CubeSarSettings(handles.I,handles.q0,handles.Wo);


% --- Executes on button press in perturbation_button.
function perturbation_button_Callback(hObject, eventdata, handles)
% hObject    handle to perturbation_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PerturbationSettings(handles.Tdx,handles.Tdy,handles.Tdz,handles.A,...
        handles.w1,handles.w2,handles.w3);


% --- Executes on selection change in controller_popupmenu.
function controller_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to controller_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns controller_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from controller_popupmenu


% --- Executes during object creation, after setting all properties.
function controller_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to controller_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in controller_button.
function controller_button_Callback(hObject, eventdata, handles)
% hObject    handle to controller_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
v = get(handles.controller_popupmenu, 'value');
switch v
    case 1
        ControllerSettings(handles.qd, handles.Wd,handles.Keye,handles.Peye);
    case 2
        BoskSettings(handles.qd, handles.Wd, handles.delta, handles.gamma,...
            handles.Umax, handles.k0);
    otherwise
        disp('other value')
end



% --- Executes on button press in run_button.
function run_button_Callback(hObject, eventdata, handles)
% hObject    handle to run_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% LS2125204: Get simulation parameters
ti = str2double(get(handles.initTime_edit, 'string'));
tf = str2double(get(handles.finalTime_edit, 'string'));
step = str2double(get(handles.step_edit, 'string'));
if isnan(ti)
    errordlg('Invalid value of Initial Time.', 'Invalid Value', 'modal');
elseif isnan(tf)
    errordlg('Invalid value of Final Time.', 'Invalid Value', 'modal');    
elseif isnan(step)
    errordlg('Invalid value of Step.', 'Invalid Value', 'modal');   
elseif ti >= tf
    errordlg('Initial Time must be lower than Final Time.', 'Invalid Value', 'modal');
else
    % LS2125204: Create time vector
    handles.t=ti:step:tf;
    handles.n=length(handles.t);
    % LS2125204: Create set-point array
    % Desired attitude array   
    qd_Array=[handles.qd(1)*(ones(handles.n,1)),handles.qd(2)*...
        (ones(handles.n,1)),handles.qd(3)*(ones(handles.n,1)),...
        handles.qd(4)*(ones(handles.n,1))]'; 
    %Desired angular rate array
    wd_Array=[handles.Wd(1)*(ones(handles.n,1)),handles.Wd(2)*...
        (ones(handles.n,1)), handles.Wd(3)*(ones(handles.n,1))]';                    
    %Initial control torque (Nm)
    ui=zeros(3,1);        
    if (handles.i==0)
        handles.Td=zeros(3,handles.n);
    elseif (handles.i==1)
        handles.Td=[handles.Tdx*(ones(handles.n,1)),handles.Tdy*...
            (ones(handles.n,1)),handles.Tdz*(ones(handles.n,1))]';
    elseif (handles.i==2)    
        handles.Td=handles.A*[sin(handles.w1*handles.t);sin(handles.w2...
            *handles.t);sin(handles.w3*handles.t)];
    end
    %State Space Vector
    x=[handles.q0;handles.Wo];
    
    v = get(handles.controller_popupmenu, 'value');
    switch v
        case 1
        %Perform Simulation
        [handles.angArray,handles.xArray,handles.uiArray,handles.TdArray,...
        handles.EULERINT,handles.ASCCT,T,ts] = Feedback_simulation(handles.Td,...
        handles.Peye,handles.Keye,handles.n,handles.t,...
        handles.I,qd_Array,wd_Array,ui,x);
        handles.kArray=[];
        handles.c1=1;
        case 2
        [handles.kArray,handles.angArray,handles.xArray,handles.uiArray,...
        handles.TdArray,handles.EULERINT,handles.ASCCT,T,ts] = ...
        Boskovic_simulation(handles.Td,handles.gamma,handles.delta,...
        handles.k0,handles.n,handles.t,handles.Umax,handles.I,qd_Array,...
        wd_Array,ui,x);
        handles.c1=2;
        %handles.graphic1_popupmenu.String = 'K';
        otherwise
        disp('other value')
    end
    
    
    %Plots
    axes(handles.graphic1_axes);
    plot(handles.t(1:handles.n-1),180/pi*handles.angArray)
    grid on; xlabel('Time(s)');
    ylabel('Euler Angles(deg)');
    legend('Roll','Pitch','Yaw');
    
    axes(handles.graphic2_axes);    
    plot(handles.t(1:handles.n-1),handles.xArray(5:7,:))
    grid on; xlabel('Time(s)');
    legend('X','Y','Z');
    ylabel('Angular Rates Error(rad/s)');
    
    %Enable buttons
    set(handles.graphic1_button,'Enable','on');
    set(handles.graphic2_button,'Enable','on');
    
    %Set push buttons
    set(handles.graphic1_popupmenu, 'value', 1);
    set(handles.graphic2_popupmenu, 'value', 3);
    
    %LS2125204: Update Controller performance parameters
    set(handles.settime_text, 'String', num2str(ts,'%.3f'));
    set(handles.eulerint_text, 'String', num2str(handles.EULERINT(end),'%.3f'));
    set(handles.ascct_text, 'String', num2str(handles.ASCCT(end),'%.3f'));
    set(handles.compCost_text, 'String', num2str(mean(T)));
end
%Update handles structure
guidata(hObject, handles)
