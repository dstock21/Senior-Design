function varargout = DeadSimulationPage(varargin)
% DEADSIMULATIONPAGE MATLAB code for DeadSimulationPage.fig
%      DEADSIMULATIONPAGE, by itself, creates a new DEADSIMULATIONPAGE or raises the existing
%      singleton*.
%
%      H = DEADSIMULATIONPAGE returns the handle to a new DEADSIMULATIONPAGE or the handle to
%      the existing singleton*.
%
%      DEADSIMULATIONPAGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DEADSIMULATIONPAGE.M with the given input arguments.
%
%      DEADSIMULATIONPAGE('Property','Value',...) creates a new DEADSIMULATIONPAGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DeadSimulationPage_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DeadSimulationPage_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DeadSimulationPage

% Last Modified by GUIDE v2.5 01-Apr-2018 22:04:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DeadSimulationPage_OpeningFcn, ...
                   'gui_OutputFcn',  @DeadSimulationPage_OutputFcn, ...
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


% --- Executes just before DeadSimulationPage is made visible.
function DeadSimulationPage_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DeadSimulationPage (see VARARGIN)

% Choose default command line output for DeadSimulationPage
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DeadSimulationPage wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DeadSimulationPage_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
xh = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'E4:F109')/100;
xa = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;
x = xa-xh;
handles.timer = timer('Name','MyTimer',               ...
                       'Period',0.1,                    ... 
                       'StartDelay',0,                 ... 
                       'TasksToExecute',inf,           ... 
                       'ExecutionMode','fixedSpacing', ...
                       'TimerFcn',{@dtimerCallback,handles.figure1});
handles.switch = 1;
axes(handles.axes2);
handles.Pos = plot(x(:,1), x(:,2), '.r', NaN, NaN, '.b');
handles.time = 0;
legend('Reference', 'Actual');
title('Gait Characterization');
xlabel('Ankle X Position (m)');
ylabel('Ankle Y Position (m)');

axes(handles.axes3);
handles.Hka = plot(NaN, NaN);
title('Knee Angle');
xlabel('Time (s)');
ylabel('Knee Angle (rad)');

 axes(handles.axes4);
handles.Hkt = plot(NaN, NaN);
title('Knee Torque');
xlabel('Time (s)');
ylabel('Knee Torque (Nm)');

 axes(handles.axes6);
handles.Hha = plot(NaN, NaN);
title('Hip Angle');
xlabel('Time (s)');
ylabel('Hip Angle (rad)');

 axes(handles.axes5);
handles.Hht = plot(NaN, NaN);
title('Hip Torque');
xlabel('Time (s)');
ylabel('Hip Torque (Nm)');

p_id = getappdata(0, 'p_id');
ts = getappdata(0, 'ts');
v = [p_id, '-', ts, '.csv'];
handles.values = csvread([p_id, '-', ts, '.csv']);

guidata(hObject,handles);
start(handles.timer);
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data = [get(handles.Pos(2, 1),'XData'), get(handles.Pos(2, 1),'YData'), get(handles.Hka,'XData'), get(handles.Hka,'YData'), get(handles.Hkt,'XData'), get(handles.Hkt,'YData')...
    , get(handles.Hha,'XData'), get(handles.Hha,'YData'), get(handles.Hht,'XData'), get(handles.Hht,'YData')];
p_id = getappdata(0, 'p_id');
d = datetime('now');
d_str = datestr(d, 30);
filename = [p_id,'-', d_str, '.csv'];
csvwrite(filename, data);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close;


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton2.
function pushbutton2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.switch == 1
    handles.switch = 0;
    stop(handles.timer);
    set(handles.pushbutton4, 'String', 'Continue');
else 
    handles.switch = 1;
    start(handles.timer);
    set(handles.pushbutton4, 'String', 'Pause');
end

guidata(hObject, handles);
