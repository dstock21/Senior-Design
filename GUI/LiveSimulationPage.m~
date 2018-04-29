 function varargout = LiveSimulationPage(varargin)
% LIVESIMULATIONPAGE MATLAB code for LiveSimulationPage.fig
%      LIVESIMULATIONPAGE, by itself, creates a new LIVESIMULATIONPAGE or raises the existing
%      singleton*.
%
%      H = LIVESIMULATIONPAGE returns the handle to a new LIVESIMULATIONPAGE or the handle to
%      the existing singleton*.
%
%      LIVESIMULATIONPAGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LIVESIMULATIONPAGE.M with the given input arguments.
%
%      LIVESIMULATIONPAGE('Property','Value',...) creates a new LIVESIMULATIONPAGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LiveSimulationPage_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LiveSimulationPage_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LiveSimulationPage

% Last Modified by GUIDE v2.5 31-Mar-2018 18:44:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @LiveSimulationPage_OpeningFcn, ...
    'gui_OutputFcn',  @LiveSimulationPage_OutputFcn, ...
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


% --- Executes just before LiveSimulationPage is made visible.
function LiveSimulationPage_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LiveSimulationPage (see VARARGIN)

% Choose default command line output for LiveSimulationPage
handles.output = hObject;


% UIWAIT makes LiveSimulationPage wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LiveSimulationPage_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Create handles that deal with simulation characteristics
handles.output = hObject;
handles.kas = [];
handles.has = [];
handles.kts = [];
handles.hts = [];
handles.xAlls = zeros(8, 1);
handles.axs = zeros(6, 1);
handles.ts = [];
handles.curr = 0;
handles.limit = 25;
handles.ref = deg2rad(qrl);
handles.errbound = 20;
temp = load('gong');
handles.sound = temp.y;
handles.Fs = temp.Fs;
handles.errthresh = 20;
handles.thickness = 4;
handles.errcounthip = 0;
handles.errcountknee = 0;
handles.errhip = 30;
handles.errknee = 30;


%Load up reference gait

xh = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'E4:F109')/100;
xa = xlsread('Winter_Appendix_data.xlsx','A1.Raw_Coordinate', 'K4:L109')/100;
x = xa-xh;
handles.timer = timer('Name','MyTimer',               ...
    'Period',0.001,                    ...
    'StartDelay',0,                 ...
    'TasksToExecute',inf,           ...
    'ExecutionMode','fixedSpacing', ...
    'TimerFcn',{@timerCallback,handles.figure1, hObject});
% or: jSlider.setPaintLabels(1);  % with both ticks and labels

% Set up serial connection
handles.arduino = setupSerial('/dev/tty.DSDTECHHC-05-DevB');

%Set up plot for for ankle position with reference gait
handles.t = 0;
axes(handles.axes2);
handles.Pos = plot(x(:,1), x(:,2), 'or', NaN, NaN, 'ob', 'markers', 8);
handles.time = 0;
legend('Reference', 'Actual');
title('Gait Characterization', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('Ankle X Position (m)','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Ankle Y Position (m)','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);

%Set up plot of leg simulation with reference gait
axes(handles.axes7);
handles.sim = plot(NaN, NaN, '-b', NaN, NaN, '.b', NaN, NaN, '-r', NaN, NaN, '.r','markers', 60);
xlim([-0.45, 0.55]);
ylim([-.9, 0.2]);
handles.Htext1 = text(handles.axes7, 0, 0, '');
handles.Htext2 = text(handles.axes7, 0, 0, '');
handles.Htext3 = text(handles.axes7, 0, 0, '');
axis equal
legend('Actual', '','Reference','');
title('Leg Simulation', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('X Relative to Hip Position','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Y Relative to Hip Position','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);


% Set up plot of Knee angle vs time
axes(handles.axes3);
handles.Hka = plot(NaN, NaN);
%ylim([-2, 2]);
title('Knee Angle', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('Time (s)','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Knee Angle (rad)','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);

% Set up plot of Knee Torque vs time
axes(handles.axes4);
handles.Hkt = plot(NaN, NaN);
title('Knee Torque', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('Time (s)','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Knee Torque (Nm)','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);

% Set up plot of Hip angle vs time
axes(handles.axes6);
handles.Hha = plot(NaN, NaN);
title('Hip Angle', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('Time (s)','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Hip Angle (rad)','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);

% Set up plot og Hip torque vs time
axes(handles.axes5);
handles.Hht = plot(NaN, NaN);
title('Hip Torque', 'FontWeight','bold', 'FontSize',30, 'Color','[0, 0.45, 0.74]');
xlabel('Time (s)','FontSize',25,'Color','[0, 0.45, 0.74]');
ylabel('Hip Torque (Nm)','FontSize',25,'Color','[0, 0.45, 0.74]');
set(findall(gca, 'Type', 'Line'),'LineWidth',handles.thickness);

% Torque slider implelemntation
handles.jSlider = javax.swing.JSlider;
[handles.hObjava, handles.hslider] =javacomponent(handles.jSlider);
set(handles.hslider, 'Parent', handles.figure1);
set(handles.hslider,'units','normalized','position', [0.845,0.92,0.11,0.051]);
set(handles.jSlider,...
    'MajorTickSpacing',20,...
    'Value',50, ...
    'PaintTicks',true,...
    'PaintLabels',true...
    );
handles.hjSlider = handle(handles.jSlider, 'CallbackProperties');
set(handles.hjSlider, 'StateChangedCallback', {@slider1_Callback,handles.figure1});
guidata(hObject,handles);
setappdata(0, 'slider', handles.jSlider)
setappdata(0, 'arduino', handles.arduino)
start(handles.timer);
varargout{1} = handles.output;
guidata(hObject, handles);



% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles, p)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Obtain value from slider and send to  arduino
slider = getappdata(0, 'slider');
arduino = getappdata(0, 'arduino');
value = slider.getValue;
fprintf(arduino, '%c', 'k');
fprintf(arduino, '%f', (value/100)*0.2);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Send set zero signal to arduino.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fprintf(handles.arduino, '%c', 'r');

% --- Save data to file.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.timer);
data = [handles.axs(1, :); handles.axs(2, :); handles.axs(3, :); handles.axs(4, :);...
    handles.axs(5, :); handles.axs(6, :); handles.xAlls(1, :); handles.xAlls(2, :);...
    handles.xAlls(3, :); handles.xAlls(4, :); handles.xAlls(5, :); handles.xAlls(6, :);...
    handles.xAlls(7, :); handles.xAlls(8, :);...
    handles.xAllrs(1, :); handles.xAllrs(2, :);...
    handles.xAllrs(3, :); handles.xAllrs(4, :); handles.xAllrs(5, :); handles.xAllrs(6, :);...
    handles.xAllrs(7, :); handles.xAllrs(8, :);handles.kas; handles.kts; handles.has; handles.hts....
    ; handles.ts];
p_id = getappdata(0, 'p_id');
d = datetime('now');
d_str = datestr(d, 30);
filename = [p_id,'-', d_str, '.csv'];
csvwrite(filename, data);
fclose(handles.arduino);
guidata(hObject,handles);
close all;


% --- Exit simulation page.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.timer);
fclose(handles.arduino);
guidata(hObject,handles);
close all;


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton2.
function pushbutton2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
