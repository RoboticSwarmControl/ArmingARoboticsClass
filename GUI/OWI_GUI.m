function OWI_GUI(Arduino)
%{

Author: Benedict Isichei

To use this GUI, the user either passes an arduino board to the function 
when calling it, or selects the arduino port in the Arduino Port menu
The main interface shows a 3D representation of the robotic arm. This
preview only works when the arm's Joints have been calibrated. 

The display tab allows the user to change the display settings of the
arm preview, and upload the previewed arm position to the physical robot.

The arduino tab is used to indicate the Robot's I/O and calibrate the 
robot's joints. View Readme for info on pins. The calibration panel on this
tab only shows up only after the pins have been set.

The automation tab allows the end User to program simple robot motions.

(NB. The pins were programmed with a MEGA board in mind, and
thus will only recognise digital pins from 2 to 53, and analog pins
from 0 to 15)

Due to the nature of DH parameters and how they build up from the base,
an index value of 1 typically corresponds to a parameter related to M5,
while an index of 5 corresponts to a parameter of M1.    
    
    TODO:
    Status update function. Maybe a status log
    Only accept suitable arduino boards:
        Uno
        Leonardo
        Mega2560
        MegaADK  
    Edit OutofRange function appropriately
    ImproperPinValues should take ardiuno as input to determine pin config(mainly for OutofRange)
    Proper braking
    Speed Control
    Multi-arm support
%}

clc
% Makes sure there's only one instance of the unbound GUI up at a time
Fig = findall(0,'Name','OWI_GUI');
if isempty(Fig)
    Fig = figure('Visible','off','Units','Pixels','ToolBar','none',...
        'Name','OWI_GUI','MenuBar','none','Numbertitle','off',...
        'WindowKeyPressFcn',@MainKeyPress_Callback);
else
    clf(Fig);
    Fig.Visible = 'off'; Fig.Units = 'Pixels';
    Fig.ToolBar = 'none'; Fig.MenuBar = 'none';
    Fig.NumberTitle = 'off';
end
if ~(nargin && isa(Arduino,'arduino'))
    %Create port menu if an arduino object isn't passed to the function
    uimenu(Fig,'Text','Arduino Port',...
        'MenuSelectedFcn', @Menu_Callback);
else
    %If an arduino object is passed to the function, bind the function
    Fig.Name = strcat('OWI_GUI (',Arduino.Port,')');
    disp(['Using arduino on ', Arduino.Port]);
end

% Setup the axes and the main tab groups
Axes1 = axes(Fig,'Units','Pixels','Position',[50,60,400,400],'Box','on');
%disableDefaultInteractivity(Axes1); %turn off rotate mode and allow
%WindowKeyPressFcn to work %comment out for older matlab versions
TabGroup = uitabgroup(Fig,'Units','Pixels','position',[470 0, 280, 500],...
    'SelectionChangedFcn',@TabChanged_Callback);
DisplayTab = uitab(TabGroup,'Title','Display');
ArduinoTab = uitab(TabGroup,'Title','Arduino Setup'); 
ProgramTab = uitab(TabGroup,'Title','Automation');
% InverseTab = uitab(TabGroup, 'Title','Inverse Kinematics');
% CamTab = uitab(TabGroup,'Title','ImageProcessing');

%% DATA MEMBERS
tolerance = 0.2; %volts
Interrupt = 0;
saveindex = 1;
Active = 1;

Data = struct;
Pins = struct;      %storage for pin tab children
Settings = struct;	%storage for settings
Program = struct;	%storage for prgram tab children
File = struct;      %temp data storage for saving and loading

%arm DH parameters
Data.input = [...
      0, -pi/2, 0, Inf;
     90,     0, 0, Inf;
    118,     0, 0, Inf;
     41,     0, 0, Inf];
%Parameter limits
Data.ArmLimits  = [...
    -3*pi/4, 3*pi/4;% M5
          0,     pi;% M4
    -3*pi/4, 3*pi/4;% M3
      -pi/3,   pi/3;% M2
          0,    45];% Gripper
%Default Pin Settings
Pins.Default = [...
    4      7    12;% pot5,Enable5,Direction5
    3      6    11;% pot4,Enable4,Direction4
    2      5    10;% pot3,Enable3,Direction3
    1      4     9;% pot2,Enable2,Direction2
    0      3     8;% pot1,Enable1,Direction1
    5     13    Active];% Interrupt,Light,High Value(boolean)
Pins.Set = 0; %Pin set flag
for count = 1:5
    Pins.Ana(count) = NaN;
    Pins.En(count) = NaN;
    Pins.Dir(count) = NaN;
    Pins.Pot.Min(count).Value = NaN;
    Pins.Pot.Min(count).Set = 0;
    Pins.Pot.Max(count).Value = NaN;
    Pins.Pot.Max(count).Set = 0;
end
Program.Config = {};

Data.Stop = uicontrol('Position',[662.5,125,70,80],'String','STOP',...
    'BackgroundColor',[.8 .3 .3],'Callback',@Stop);
Data.RevUpdate = uicontrol('String','UpdatePlot',...
    'Callback',@PlotUpdate_Callback);

%% DISPLAY TAB SETUP
ViewSettings = uipanel(DisplayTab,'Units','Pixels','Position',[10,230,155,105],...
    'Title', 'VIEWS','TitlePosition','centertop');
AxisSettings = uipanel(DisplayTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','AXIS LIMITS','TitlePosition','centertop','Visible','off');
SlideSettings = uipanel(DisplayTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','SLIDE LIMITS','TitlePosition','centertop','Visible','off');
RemoteSettings = uipanel(DisplayTab,'Units','Pixels','Position',[2 345 26 130]);
DHSettings = uipanel(DisplayTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','DH Parameters','TitlePosition','centertop','Visible','off');


%DisplayTab CHILDREN
base = 350; %base y value for sliders
for count = 1:5 %create and align display sliders
    Data.SlideReset(count) = uicontrol(DisplayTab,'UserData',count,...
        'Position',[32,base+30*(count-1)+10,20,12],'Callback',@ResetArms_Callback,...
        'String',strcat('M',num2str(6-count)),'TooltipString','Reset this joint');
    Data.SlideText(count) = uicontrol(DisplayTab,'Style','edit','UserData',count,...
        'Position',[32,base+30*(count-1)-2,20,14],'Callback',@SlideText_Callback,...
        'String',0);
    Data.PowerSlide(count) = uicontrol(DisplayTab,'Style','Slider','UserData',count,...
        'Position',[245,base+30*(count-1)+10,5,12],'Callback',@Slide_Callback,...
        'String','P','Min',0,'Max',200,'Value',100);
    Data.PowerText(count) = uicontrol(DisplayTab,'Style','edit','UserData',count,...
        'Position', [56,base+30*(count-1)+10,20,12],'Callback',@PowerText_Callback,...
        'String',100);
    Data.Slide(count) =  uicontrol(DisplayTab,'Style','slider','UserData',count,...
        'Position',[55,base+30*(count-1),200,10],'Callback',@Slide_Callback,...
        'String', 'R','Min', Data.ArmLimits(count,1),'Max',Data.ArmLimits(count,2),...
        'Value',0);
    Data.Upload(count) = uicontrol(DisplayTab,'FontSize',12,'UserData',count,...
        'position',[258,base+30*(count-1)-2,14,12],'Callback',@Upload_Callback,...
        'String','>','FontWeight','bold','TooltipString','Upload to board');        
    align([Data.SlideText(count) Data.SlideReset(count)],'Center','Fixed',1);
    align([Data.SlideText(count) Data.Slide(count) Data.Upload(count)],'None','Middle');
    align([Data.PowerText(count) Data.Upload(count)],'Right','Fixed',1);
    align([Data.PowerSlide(count) Data.PowerText(count)],'None','Top');
end
Data.Slide(5).SliderStep = [1/90 1/18];
Data.Slide(5).String = 'G';%gripper

Data.UploadAll = uicontrol(DisplayTab,'Position',[175,290,90,39],...
    'UserData',[1 2 3 4 5],'String','Upload all',...
    'Callback',@Upload_Callback,'TooltipString','Upload all Values');
Data.ResetArm = uicontrol(DisplayTab,...
    'Position',[175,260,90,20],'String','Reset Arm',...
    'Callback',@ResetArms_Callback,'UserData',6);
Data.Radians = uicontrol(DisplayTab,'Position', [175,230,90,20],...
    'String','Radians','Style','toggle','Callback',@Radian_Callback);

Terminal = uicontrol(Fig,'Style','text',...
    'Position',[480,125,180,95],'Max',2,'HorizontalAlignment','left',...
    'BackgroundColor',[.91 .91 .93],'ButtonDownFcn',@Display_ButtonDownCallback);
Settings.Axis.Reset = uicontrol(DisplayTab,'Position',[195 180 70 40],...
    'String','Reset Axis','Callback',@ResetAxis_Callback,...
    'Visible','off');
Settings.Slide.Reset = uicontrol(DisplayTab,'UserData','all',...
    'Position',[195 180 70 40],'Callback',@ResetLimits_Callback,...
    'String','Reset all Slides','Visible','off');    
Settings.Advanced1 = uicontrol(DisplayTab,'Style','radio',...
    'String','Axis','Position',[5 5 50 20],...
    'UserData','Axis','Callback',@AdvancedControls_Callback);
Settings.Advanced2 = uicontrol(DisplayTab,'Style','radio',...
    'String','Slide','Position',[30 5 50 20],...
    'UserData','Slide','Callback',@AdvancedControls_Callback);
Settings.DH = uicontrol(DisplayTab,'Style','radio',...
    'String','Show DH','Position',[55 5 50 20],...
    'UserData','DH','Callback',@AdvancedControls_Callback);

for count = 1:size(Data.input,1)
    Data.HGTrans(count) = hgtransform(Axes1);
end
for count = 1:6
    Data.GripTrans(count) = hgtransform(Axes1);
end


%VIEWS SETTINGS CHILDREN
Settings.View.YZ = uicontrol(ViewSettings,'Position',[10,10,30,20],...
    'String','YZ','Callback',@Display_Callback);
Settings.View.YZ_ = uicontrol(ViewSettings,'Position',[50,10,30,20],...
    'String','YZ''','Callback',@Display_Callback);
Settings.View.XZ = uicontrol(ViewSettings,'Position',[10,40,30,20],...
    'String','XZ','Callback',@Display_Callback);
Settings.View.XZ_ = uicontrol(ViewSettings,'Position',[50,40,30,20],...
    'String','XZ''','Callback',@Display_Callback);
Settings.View.YX = uicontrol(ViewSettings,'Position',[10,70,30,20],...
    'String','YX','Callback',@Display_Callback);
Settings.View.Reset = uicontrol(ViewSettings,'Position',[50,70,90,20],...
    'String','RESET','Callback',@Display_Callback);
Settings.View.Rotate = uicontrol(ViewSettings,'Style','toggle','Position',[90,40,50,20],...
    'String','ROTATE','Callback',@Display_Callback);
Settings.View.Iso = uicontrol(ViewSettings,'Position',[90,10,50,20],...
    'String','ISO','Callback',@Display_Callback);


%AXIS SETTINGS CHILDREN
Settings.Axis.TopLabel = uicontrol(AxisSettings,'Style','text',...
    'String','Min : Max','Position',[10,83,40,20]);
dim = 'xyz';
for count = 1:3
    Settings.Axis.Min(count) = uicontrol(AxisSettings,'Style','edit','Userdata',count,...
        'Position',[10,10+(count-1)*30,50,20]);
    Settings.Axis.Max(count) = uicontrol(AxisSettings,'Style','edit','Userdata',count,...
        'Position',[65,10+(count-1)*30,50,20]);
    Settings.Axis.Set(count) = uicontrol(AxisSettings,'Userdata',count,...
        'Position',[120,10+(count-1)*30,50,20],'Callback',@AxisLimit_Callback,...
        'String', strcat('Set ',dim(count)));
end


%SLIDE SETTINGS CHILDREN
Settings.Slide.Menu = uicontrol(SlideSettings,'Style','pop',...
    'Position',[10,60,50,20],'String',{'M5';'M4';'M3';'M2';'M1'});
Settings.Slide.ShowBoxes = uicontrol(SlideSettings,'Style','toggle',...
    'Position',[70,56,100,25],'String','View Slide Limit Boxes',...
    'Callback',@ViewSlider_Callback);
Settings.Slide.TopLabel = uicontrol(SlideSettings,'Style','text',...
    'String','Min : Max','Position',[10,23,40,20],'Visible','off');
Settings.Slide.Reset2 = uicontrol(SlideSettings,'Userdata','single',...
    'Position',[70,35,100,15],'String','Reset Slide',...
    'Callback',@ResetLimits_Callback);
for count = 1:5
    Settings.Slide.Min(count) = uicontrol(SlideSettings,'Style','edit','UserData',count,...
        'Position',[10,10,50,20],'Visible','off');
    Settings.Slide.Max(count) = uicontrol(SlideSettings,'Style','edit','UserData',count,...
        'Position',[65,10,50,20],'Visible','off');
    Settings.Slide.Label(count) = uicontrol(SlideSettings,'UserData',count,...
        'Position',[120,10,50,20],'Callback',@SlideLimit_Callback,...
        'String','Limit Slider','Visible','off');
end

%REMOTESETTINGS CHILDREN
for count = 1:5
    Settings.Remote(count) = uicontrol(RemoteSettings,'Userdata',count,...
        'Position',[2,3+25*(count-1),20,20],'Callback',@Remote_Callback,...
        'String',strcat('M',num2str(6-count)),'BackgroundColor',[.9 .9 1],...
        'KeyPressFcn',@RemoteKeyPress_Callback,'ForegroundColor',[.5 .6 .7]);
end

%DH CHILDREN
Data.DHTable = uitable(DHSettings,'Data',[Data.input(:,1:3) [Data.Slide(1).Value;Data.Slide(2).Value;Data.Slide(3).Value;Data.Slide(4).Value]],...
    'Position',[4,20,170,64],'RowName',{'M2','M3','M4','M5'},...
    'ColumnName',{'R','alpha','d','theta'},'ColumnWidth',{66,68,66,69});

%% ARDUINO TAB SETUP
PinSettings = uipanel(ArduinoTab,'Units','Pixels','Position',[5,290,270,210],...
    'Title','Pin Settings');
CalPanel = uipanel(ArduinoTab,'Units','Pixels','Position',[5 45 270 235],...
    'Title','Calibration','Visible','off');


%PINSETTINGS CHILDREN
OutBitSetting = uibuttongroup(PinSettings,'Units','Pixels',...
    'Position',[5 160 190 30],'Title','What value activates the motors?');
Settings.Pin.High = uicontrol(OutBitSetting,'Style','radio',...
    'Position',[5 3 50 14],'String','HIGH');
Settings.Pin.Low = uicontrol(OutBitSetting,'Style','radio',...
    'Position',[65 3 50 14],'String','LOW');

Settings.Pin.EnableLabel = uicontrol(PinSettings,'Style','text',...
    'Position',[155,125,15,15],'String','En',...
    'TooltipString','These values indicate what pins enable the motors');
Settings.Pin.DirectionLabel = uicontrol(PinSettings,'Style','text',...
    'Position',[176,125,15,15],'String','Dir',...
    'TooltipString','These values indicate what direction the motors move');
base = 25; offset = 22;
for count = 1:5
    Settings.Pin.ALabel(count) = uicontrol(PinSettings,'Style','text','UserData',count,...
        'Position',[5,base+offset*(count-1),45,15],'HorizontalAlignment','left',...
        'String',strcat('M',num2str(6-count),' Pot Pin'));
    Settings.Pin.APin(count) = uicontrol(PinSettings,'Style','edit','UserData',count,...
        'Position',[23,base+offset*(count-1),20,15]);
    Settings.Pin.Label(count) = uicontrol(PinSettings,'Style','text','UserData',count,...
        'Position',[36,base+offset*(count-1),65,15],'HorizontalAlignment','Right',...
        'String',strcat('M',num2str(6-count),' Output Pins'));
    Settings.Pin.En(count) = uicontrol(PinSettings,'Style','edit','UserData',count,...
        'Position',[155,base+offset*(count-1),20,15]);
    Settings.Pin.Dir(count) = uicontrol(PinSettings,'Style','edit','UserData',count,...
        'Position',[176,base+offset*(count-1),20,15]);
    align ([Settings.Pin.ALabel(count) Settings.Pin.APin(count) Settings.Pin.En(count) ...
        Settings.Pin.Dir(count) Settings.Pin.Label(count)], 'Fixed',1,'Middle');
end

Settings.Pin.InterruptLabel = uicontrol(PinSettings,'Style','text','UserData',7,...
        'Position',[5,base+offset*(5),45,15],'HorizontalAlignment','Left',...
        'String','Interrupt');
Settings.Pin.Interrupt = uicontrol(PinSettings,'Style','edit','UserData',7,...
        'Position',[36,base+offset*(5),20,15]);
Settings.Pin.LightLabel = uicontrol(PinSettings,'Style','text','UserData',6,...
        'Position',[77,base+offset*(5),40,15],'HorizontalAlignment','Left',...
        'String','Light Pin');
Settings.Pin.Light = uicontrol(PinSettings,'Style','edit','UserData',6,...
        'Position',[120,base+offset*(5),20,15]);
align([Settings.Pin.EnableLabel Settings.Pin.Label(5)],'Fixed',2,'Fixed',1);
align([Settings.Pin.DirectionLabel Settings.Pin.EnableLabel],'Fixed',2,'Middle');
align([Settings.Pin.LightLabel Settings.Pin.Light],'Fixed',1,'Bottom');
align([Settings.Pin.InterruptLabel Settings.Pin.Interrupt], 'Fixed',1,'Botton');

Settings.Pin.Set = uicontrol(PinSettings,'String','Apply All Values',...
    'Position',[5,3,260,20],'Callback', @SetPin_Callback);    
Settings.Pin.Save = uicontrol(PinSettings,'Position',[200 170 50 20],...
    'Callback',@Save_Callback,'String','Save Pins');
Settings.Pin.Load = uicontrol(PinSettings,'Position',[200 145 50 20],...
    'Callback',@Load_Callback,'String','Load Pins');


%CALPANEL CHILDREN
CalSettings = uibuttongroup(CalPanel,'Units','Pixels','Position',[0 188 270 50]);
for count = 1:5
    Settings.Cal.Button(count) = uicontrol(CalSettings,'Style','toggle','UserData',count,...
        'Position',[1+53*(count-1),5,50,20],'Callback',@CalButton_Callback,...
        'String',strcat('M',num2str(6-count)));
end
Settings.Cal.Orient = uicontrol(CalPanel,'Units','Pixels','String','Orient',...
    'Position',[5 30 30 20],'Callback',@Orientation_Callback);
Settings.Cal.Set = uicontrol (CalPanel,'Units','Pixels',...
    'Position',[40 30 30 20],'String','Set','Callback',@Calibrate_Callback);
Settings.Cal.ReadingLabel = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[80 30 45 20],'String','Reading:');
Settings.Cal.Reading = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[125 30 50 20],'String','--','BackgroundColor',[.98 .98 .98]);
Settings.Cal.Save = uicontrol(CalPanel,'Position',[190 30 70 20],...
    'Callback',@Save_Callback,'String','Save Calibration');
Settings.Cal.Load = uicontrol(CalPanel,'Position',[190 5 70 20],...
    'Callback',@Load_Callback,'String','Load Calibration');

%% PROGRAM TAB SETUP
ClosedLoopPanel = uipanel(ProgramTab,'Title','Closed Loop Automation');
% OpenLoopPanel = uipanel(ProgramTab,'Visible','off','Title','Open Loop Automation');
% Program.Switch = uicontrol(ProgramTab,'Position',[100 5 70 25],...
%     'Callback',@ProgramSwitch_Callback,'String','Switch Mode','UserData',0);


%CLOSED LOOP CHILDREN
Program.Append = uicontrol(ClosedLoopPanel,'Position',[54 422 70 60],...
    'String','Save Config','Callback',@ProgramSave_Callback,'Tag','Append');
Program.Display = uicontrol(ClosedLoopPanel,'Position',[54 334 70 60],...
    'String','Show Motion','Callback',@ProgramMove_Callback,'Tag','Display',...
    'Style','togglebutton');
Program.Delete = uicontrol(ClosedLoopPanel,'Position',[54 250 70 60],...
    'String','Delete last Config','Callback',@ProgramSave_Callback,'Tag','Delete');
Program.List = uicontrol(ClosedLoopPanel,'Style','list','Position',[134 250 50 230],...
    'Callback',@ProgramList_Callback,'Tag','List','Max',2);
Program.Save = uicontrol(ClosedLoopPanel,'Position',[195 422 70 60],...
    'Callback',@Save_Callback,'String','Save Motion');
Program.Load = uicontrol(ClosedLoopPanel,'Position',[195 334 70 60],...
    'Callback',@Load_Callback,'String','Load Motion');
Program.Reset = uicontrol(ClosedLoopPanel,'Position',[195 250 70 60],...
    'String','Reset Motion','Callback',@ProgramSave_Callback,'Tag','Reset');
Program.Upload = uicontrol(ClosedLoopPanel,'Position',[10 225 255 20],...
    'UserData',[1 2 3 4 5],'String','Upload Motion',...
    'Callback',@ProgramMove_Callback,'Tag','Upload');


%OPEN LOOP CHILDREN

%% DRAWING ITEMS
axis square; view(-150,30);% rotate3d on;
defaultColor = [231 188 34]./255;
%Base1
[Vertices, Faces , ~, ~]= stlReadBinary('Models/Base_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',defaultColor,'Parent',Data.HGTrans(1),'EdgeColor',defaultColor/2);
%Base2
[Vertices, Faces , ~, ~]= stlReadBinary('Models/Base_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(1));
% Arm1
[Vertices, Faces , ~, ~]= stlReadBinary('Models/arm1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(2),'FaceColor',[.1 .1 .1]);
%Arm2_1
[Vertices, Faces, ~, ~]= stlReadBinary('Models/arm2_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(3));
%Arm2_2
[Vertices, Faces, ~, ~]= stlReadBinary('Models/arm2_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',defaultColor,'Parent',Data.HGTrans(3),'EdgeColor',defaultColor/2);
%Gripper
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(4),'FaceColor',[.1 .1 .1]);
%Gripper_1  
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(1),'FaceColor',[.19 .19 .19]);
%Gripper_2
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(2),'FaceColor',[.19 .19 .19]);
%Gripper_3 and 4
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(3),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(4),'FaceColor',[.19 .19 .19]);
%Gripper_5 and 6
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_4l.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(5),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_4r.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(6),'FaceColor',[.19 .19 .19]);
%Battery
[Vertices, Faces, ~, ~]= stlReadBinary('Models/battery.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1]);
%Table
TableTrans = hgtransform('Parent',Axes1,'Matrix',makehgtform('translate',[0 0 -70]));
rectangle('Position',[-300 -300 600 600],'Parent',TableTrans,...
    'Curvature',[1,1],'FaceColor',[.8 .8 .8]);
Fig.Position = [100 100 750 500];

%% NORMALIZING UI ELEMENTS and initial run
Fig.Units = 'normalized';
Axes1.Units = 'normalized';
TabGroup.Units = 'normalized';
uiitems = findobj('Type','uicontrol','-or','Type','uibuttongroup','-or','Type','uipanel','-or','Type','uitable');
for count = 1:size(uiitems,1)
    uiitems(count,1).Units = 'normalized';
end
uicontrols = findobj('Type','uicontrol');
for count = 1:size(uicontrols,1)
    uicontrols(count,1).FontUnits = 'normalized';
end
Fig.Position = [0.15 0.15 .7 .7];
RemoteSettings.Position = [2/280 345/500 26/280 152/500];
Settings.Advanced1.FontUnits = 'points';
Settings.Advanced2.FontUnits = 'points';
Settings.DH.FontUnits = 'points';
ResetAxis_Callback;
ResetLimits_Callback(Settings.Slide.Reset);
Update(Data);
%setting default values
for count = 1:5
    Settings.Pin.APin(count).String = Pins.Default(count,1);
    Settings.Pin.En(count).String = Pins.Default(count,2);
    Settings.Pin.Dir(count).String = Pins.Default(count,3);
    Data.SlideText(count).FontUnits = 'points';
    Data.PowerText(count).FontUnits = 'points';
end
Settings.Pin.Interrupt.String = Pins.Default(6,1);
Settings.Pin.Light.String = Pins.Default(6,2);
Settings.Pin.High.Value = Pins.Default(6,3);
Settings.Pin.Low.Value = ~Settings.Pin.High.Value;
Fig.Visible = 'on'; %show the figure

%% CALLBACKS

    %Shows or hides advanced controls not meant for the general user
    function AdvancedControls_Callback(source,~)
        %Called by Settings.Advanced1 or Advanced2 in the DisplayTab
        if strcmp(source.Units,'normalized')
            xmax = 280;
            ymax = 500;
        else
            xmax = 1;
            ymax = 1;
        end
        if source.Value
            switch source.UserData
                case 'Axis'
                    AxisSettings.Visible = 'on';
                    Settings.Axis.Reset.Visible = 'on';
                    align([Data.Radians source Settings.Axis.Reset],'Right','None')
                    Settings.Advanced2.Visible = 'off';
                    Settings.DH.Visible = 'off';
                case 'Slide'
                    SlideSettings.Visible = 'on';
                    Settings.Slide.Reset.Visible = 'on';
                    align([Data.Radians source Settings.Slide.Reset],'Right','None')
                    Settings.Advanced1.Visible = 'off';
                    Settings.DH.Visible = 'off';
                case 'DH'
                    DHSettings.Visible = 'on';
                    Settings.Advanced1.Visible = 'off';
                    Settings.Advanced2.Visible = 'off';
            end
            source.Position = [195/xmax 97/ymax 70/xmax 80/ymax];%[195 97 80 80];
            align([Data.Radians source],'Right','None')
            Data.Stop.Position = [662.5/750,12/500,70/750,80/500];
            source.Style = 'toggle';
            source.String = 'Close Panel';
        else
            DHSettings.Visible = 'off';
            AxisSettings.Visible = 'off';
            SlideSettings.Visible = 'off';
            Settings.Axis.Reset.Visible = 'off';
            Settings.Slide.Reset.Visible = 'off';
            Settings.Advanced1.Visible = 'on';
            Settings.Advanced2.Visible = 'on';
            Settings.DH.Visible = 'on';
            if strcmp(source.UserData,'Axis')
                source.Position = [ 5/xmax 5/ymax 50/xmax 20/ymax];
            source.String ='Axis';
            elseif strcmp(source.UserData,'Slide')
                source.Position = [30/xmax 5/ymax 50/xmax 20/ymax];
            source.String ='Slide';
            else
                source.Position = [55/xmax 5/ymax 50/xmax 20/ymax];
                source.String = 'DH';
            end
            source.Style = 'radio';
            Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
        end
    end
    
    %sets the x, y or z limits.
    function AxisLimit_Callback(source,~)
        %Called by Settings.Axis.Set<> in the AxisLimit uipanel
        val = source.UserData;
        min = str2double(Settings.Axis.Min(val).String);
        max = str2double(Settings.Axis.Max(val).String);
        switch val
            case 1
                xlim([min max]);
            case 2 
                ylim([min max]);
            case 3
                zlim([min max]);
        end
    end
    
    %Opens the Calibration Menu for the selected Motor
    function CalButton_Callback(source,~)
        %Called by Settings.Cal.Button in the CalSettings uipanel
        val = source.UserData;
        if ~(Pins.Pot.Min(val).Set && Pins.Pot.Max(val).Set)
            Pins.Pot.Min(val).Set = 0;
            Pins.Pot.Max(val).Set = 0;
            Status(strcat('Click Orient to Start Calibrating M',num2str(6-val)));
            Settings.Cal.Reading.String = '--';
            Settings.Cal.Set.Visible = 'off';
            Settings.Cal.Orient.Visible = 'on';
            Settings.Cal.Reading.String = analogRead(Arduino,Pins.Ana(val));
        else
            Status(strcat('M',num2str(6-val),' has been calibrated already. click Reset to recalibrate'));
            Status(', or select a different Motor to calibrate');            
            Settings.Cal.Set.Visible = 'on';
            Settings.Cal.Orient.Visible = 'off';
            Settings.Cal.Set.String = 'Reset';
            Settings.Cal.Reading.String = analogRead(Arduino,Pins.Ana(val));
        end
    end
    
    %Sets the Min or Max value for the selected motor
    function Calibrate_Callback(source,~)
        %Called by Settings.Cal.Set on the CalSettings uipanel
        val = CalSettings.SelectedObject.UserData;
        Motor = strcat('M',num2str(6-val));
        if ~Pins.Pot.Min(val).Set
            Pins.Pot.Min(val).Value = analogRead(Arduino,Pins.Ana(val));
            Pins.Pot.Min(val).Set = 1;
            Settings.Cal.Reading.String = Pins.Pot.Min(val).Value;
            Status(strcat(Motor,' Min has been Set, Click Orient to proceed'));
        elseif ~Pins.Pot.Max(val).Set
            Pins.Pot.Max(val).Value = analogRead(Arduino,Pins.Ana(val));
            Pins.Pot.Max(val).Set = 1;
            Settings.Cal.Reading.String = Pins.Pot.Max(val).Value;
            Status(strcat(Motor,' Max been calibrated. Click Reset to recalibrate'));
            Status(' or select a different Motor to calibrate');
            source.String = 'Reset';
            Settings.Cal.Orient.Visible = 'off';
        else
            Pins.Pot.Min(val).Set = 0;
            Pins.Pot.Max(val).Set = 0;
            source.String = 'Set';
            Settings.Cal.Orient.Visible = 'on';
            Status(strcat('Click Orient to begin calibration'));
        end
    end

    function Display_ButtonDownCallback(source,~)
        source.String = '';
    end

    %Initializes the arduino
    function Initialize_Arduino(source,~)
        Arduino = arduino(source.Text);
        if isa(Arduino,'arduino')
            Currentfig = gcf;
            Currentfig.Name = strcat('OWI_GUI (',Arduino.Port,')');
        end
        source.Parent.Visible = 'off';
    end

    %Loads File from disk
    function Load_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                uiopen('Settings/Pins/Pins.mat');
                if numel(File) == 4
                    for i = 1:5
                        Settings.Pin.APin(i).String = File{1}.Ana(i);
                        Settings.Pin.En(i).String = File{1}.En(i);
                        Settings.Pin.Dir(i).String = File{1}.Dir(i);                        
                    end
                    Settings.Pin.Light.String = File{2};
                    Settings.Pin.Interrupt.String = File{3};
                    Settings.Pin.High.Value = File{4};
                    Settings.Pin.Low.Value = ~Settings.Pin.High.Value;
                    File = [];
                    Status('Pin info loaded');
                else
                    Status('Invalid file');
                end
            case 'Calibration'
                uiopen('Settings/Cal/Calibration.mat');
                if numel(File) == 1
                    Pins.Pot(:) = File{1};
                    File = [];
                    Status('Calibration file loaded');
                else
                    Status('Invalid file');
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                uiopen('Settings/Move/Movement.mat');
                if size(File,1) == 1
                    Program.Config = {Program.Config{:,:}, File{:,:}};
                    File = [];
                    for i = 1:size(Program.Config,2)
                        Program.List.String{i} = strcat('Config ',num2str(i));
                    end
                    saveindex = size(Program.Config,2)+1;
                else
                    Status('Invalid file');
                end
        end
    end
    
    %Main Keypress callback for the program
    function MainKeyPress_Callback(~,event)
        Joints = {'5','4','3','2','1'};
        motor = find(strcmp(event.Key,Joints));%get motor index
        if motor
                if strcmp(event.Modifier,'control')
                    Status(strcat('Motor_', num2str(6-motor), ' Selected'));
                    uicontrol(Settings.Remote(motor));
                    Remote_Callback(Settings.Remote(motor));
                else
                    Status(strcat('Slider_ ', num2str(6-motor), ' Selected'));
                    uicontrol(Data.Slide(motor));
                end
        end
    end

    %Updates the availiable COM Ports
    function Menu_Callback(source,~)
        delete(source.Children);
        items = seriallist;%["one" "two" "three"];
        for i= 1:numel(items)
        	uimenu(source, 'Text', items(i),...
                'MenuSelectedFcn', @Initialize_Arduino);
        end
    end
    
    %Shows the end user the proper orientation required to calibrate
    %the robot's physical readings
    function Orientation_Callback(~,~)
        %Called by Settings.Cal.Orient on the CalSettings uipanel
        val = CalSettings.SelectedObject.UserData;
        if Pins.Set
            switch val
                case 1
                    if ~Pins.Pot.Min(val).Set
                        Config([-135 0 0 0 0])
                        Terminal.String = 'Move the robot to the configuration shown on the left, then click Set';
                    else
                        Config([135 0 0 0 0]);
                        Terminal.String = 'Now move the robot to the New configuration shown , then click Set';
                    end
                    view(-90,90)
                case 2
                    if ~Pins.Pot.Min(val).Set
                        Config([0 0 90 0 0]);
                        Terminal.String = 'For this calibration, try to get the arm vertical with the base of the housing for M3 touching the top of the battery pack, then click Set';
                    else
                        Config([0 180 -90 0 0]);
                        Terminal.String = 'For this Calibration, move M4 down until the joint Clicks, then click Set';
                    end
                    view(180,0)
                case 3
                    if ~Pins.Pot.Min(val).Set
                        Config([0 180 -135 0 0]);
                        Terminal.String = 'For this calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    else
                        Config([0 0 135 0 0]);
                        Terminal.String = 'For this Calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    end
                    view(180,0)
                case 4
                    if ~Pins.Pot.Min(val).Set
                        Config([0 90 0 -60 0]);
                        Terminal.String = 'For this calibration, turn M2 until it clicks';
                    else
                        Config([0 90 0 60 0]);
                        Terminal.String = 'For this Calibration, turn M2 until it clicks';
                    end
                    view(180,0)
                case 5
                    if ~Pins.Pot.Min(val).Set
                        Config([0 90 0 0 0]);
                        Terminal.String = 'For this calibration, turn M1 until it clicks';
                    else
                        Config([0 90 0 0 45]);
                        Terminal.String = 'For this Calibration, turn M1 until it clicks';
                    end
                    view(-90,0)
            end
            Settings.Cal.Set.Visible = 'on';
            Settings.Cal.Set.String = 'Set';
        else
            Status('Please Setup Pins');
        end
    end
    
    %Updates the plot with the physical robot's current orientation
    function PlotUpdate_Callback(~,~)
        %called by Data.UpdatePlot in the Main Figure
        if  CheckCal(Pins.Pot)
            Config(GetRobotConfig(Arduino,Pins,Data.ArmLimits,Data.Slide,Data.Radians));
        else
            Status('Can''t Update plot yet');
            Status('please calibrate pins');
        end
    end
    
    %Changes the arm Movement power
    function PowerText_Callback(source,~)
        Data.PowerSlide(source.UserData).Value = str2double(source.String);        
    end
    
    %Callback for displaying a motion Config
    function ProgramList_Callback(source,~)
        Config(Program.Config{source.Value});
    end
    
    %Callback for buttons in the program tab pertaining to moving the arm
    function ProgramMove_Callback(source,~)
        %Called by Program.Display and Program.Upload in the RemoteTab
        if numel(Program.Config)
            if strcmp(source.Tag,'Display')  
                for i = 1:saveindex-1
                    Config(Program.Config{i});
                    pause(.5);
%                     Config(Program.Config{i},'slowly')
                    if ~Program.Display.Value
                        break
                    end
                end
                Program.Display.Value = 0;
            elseif strcmp(source.Tag,'Upload')
                if  CheckCal(Pins.Pot)
                    Status('Uploading');
                    for i = 1:saveindex-1
                        Config(Program.Config{i});
                        Upload_Callback(source);
                    end
                    Status('Done');
                else
                    Status('Calibrate pins first');
                    Status('It is required');
                end
            end
        else
            Status('No saved motion');
        end
    end
    
    %Callback for buttons in the program tab pertaining to motion data
    function ProgramSave_Callback(source,~)
        %Called by Program.Delete, Program.Append and Program.Reset
        if strcmp(source.Tag,'Append')
            Program.Config{saveindex} = GetPlotConfig(Data.Slide,Data.Radians);
            Status(Program.Config{saveindex});
            Status(strcat('Config ',num2str(saveindex),' saved'));
            Program.List.String{saveindex} = strcat('Config ',num2str(saveindex));
            saveindex = saveindex + 1;
        elseif strcmp(source.Tag,'Delete')&& saveindex>2
            saveindex = saveindex -1;
            Config(Program.Config{saveindex-1});
            Program.List.String{saveindex} = [];
            Status(strcat('Config ',num2str(saveindex),' deleted'));
        else
            saveindex = 1;
            Program.Config = {};
            Program.List.String = {};
            Program.List.Value = saveindex;
            Config([0 0 0 0 0]);
            Status('Motion cancelled');
        end
    end
    
    %Changes the Angles shown next to the sliders to degrees or radians
    function Radian_Callback(source,~)
        %Called by Data.Radians in the DisplayTab
        for i = 1:4
            Data.SlideText(i).String = Data.Slide(i).Value.*RadCheck(i,Data.Slide,Data.Radians);
        end
        if source.Value
            source.String = 'Degrees';
        else
            source.String = 'Radians';
        end
    end
    
    %Informs the user what buttons move the selected motor
    function Remote_Callback(source,~)
        %called by Settings.Remote<> in the RemoteSettings uipanel
        if Pins.Set
            switch source.String
                case 'M5'
                    Status('Use the left and right arrow keys to move.');
                    Status('Push any other key to stop');
                otherwise
                    Status('Use the up and down arrow keys to move.');
                    Status('Push any other key to stop');
            end
        else
            Status('Set Arduino pins first');
        end
    end
    
    %Moves the robot arm using keyboard input based on the motor selected
    function RemoteKeyPress_Callback(~,event)
        %Called by Settings.Remote<> in the RemoteSettings uipanel
        if Pins.Set
            if strcmp(event.Source.String,'M5')
                options = {'rightarrow','leftarrow'}; %if M5 is selected, use left/right for movement
            elseif any(strcmp(event.Source.String,{'M1','M3'}))
                options = {'downarrow','uparrow'};  %if M1 or M3 is selected use up/down and reverse direction
            else
                options = {'uparrow','downarrow'}; %uses these options for the others
            end
            key = event.Key;
            val = event.Source.UserData;
            EmStop(Arduino,Active,Pins.En,val);
            if strcmp(key,options{1,1})
                move(val,'+');
            elseif strcmp(key,options{1,2})
                move(val,'-');
            else
                PlotUpdate_Callback();
            end
        else
            Status('Please setup pins first');
        end
    end
    
    %Resets all the robot's arms
    function ResetArms_Callback(source,~)
        %Called by Data.ResetArm or Data.SlideLable<> in the DisplayTab
        val = source.UserData;
        if val < 6
            Data.Slide(val).Value = 0;
            Data.SlideText(val).String = 0;
            Update(Data)
        else
            for i=1:5
                ResetArms_Callback(Data.SlideReset(i));
            end
        end
    end
    
    %resets the axis limits
    function ResetAxis_Callback(~,~)
        %Called by Settings.Axis.Reset in the DisplayTab
        axis([-350 350 -350 350  -75 625]);
%         axis([200 350 -75 75  -75 75]);
        Settings.Axis.Min(1).String = -350;
        Settings.Axis.Max(1).String = 350;
        Settings.Axis.Min(2).String = -350;
        Settings.Axis.Max(2).String = 350;
        Settings.Axis.Min(3).String = -75;
        Settings.Axis.Max(3).String = 625;
    end
    
    %Resets the current slide limits or all slide limits
    function ResetLimits_Callback(source,~)
        %Called by Settings.Slide.Reset and Reset2 in the SlideLimits
        %uipanel
        switch source.UserData
            case 'single'
                val = Settings.Slide.Menu.Value;
                Data.Slide(val).Min = Data.ArmLimits(val,1);
                Data.Slide(val).Max = Data.ArmLimits(val,2);
                Settings.Slide.Min(val).String = Data.Slide(val).Min*RadCheck(val,Data.Slide,Data.Radians);
                Settings.Slide.Max(val).String = Data.Slide(val).Max*RadCheck(val,Data.Slide,Data.Radians);
                Data.Slide(val).Value = 0;
                Data.SlideText(val).String = 0;
            case 'all'
                for i = 1:5
                    Data.Slide(i).Min = Data.ArmLimits(i,1);
                    Data.Slide(i).Max = Data.ArmLimits(i,2);
                    Settings.Slide.Min(i).String = Data.Slide(i).Min*RadCheck(i,Data.Slide,Data.Radians);
                    Settings.Slide.Max(i).String = Data.Slide(i).Max*RadCheck(i,Data.Slide,Data.Radians);
                end
                ResetArms_Callback(Data.ResetArm);
        end
    end
    
    %Saves data to disk
    function Save_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                if Pins.Set
                    File = [];
                    File{1} = Pins;
                    File{2} = str2double(Settings.Pin.Light.String);
                    File{3} = str2double(Settings.Pin.Interrupt.String);
                    File{4} = Active;
                    uisave('File','Settings/Pins/Pins.mat')
                    File = [];
                else
                    Status('Please set pins first');
                end
            case 'Calibration'
                if CheckCal(Pins.Pot)
                    File = {Pins.Pot(:)};
                    uisave('File','Settings/Cal/Calibration.mat')
                    File = [];
                else
                    Status('please calibrate pins first');
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                if numel(Program.Config)
                    File = Program.Config;
                    uisave('File','Settings/Move/Movement')
                    File = [];
                else
                    Status('Start a motion first');
                end
        end
    end
    
    %Sets the Pin ports for the arduino
    function SetPin_Callback(~,~)
        %Called by Settings.Cal.Set in the PinSettings uipanel
        Status('');
        if exist('Arduino', 'var') && isa(Arduino,'arduino')
            Apin = {Settings.Pin.APin.String Settings.Pin.Interrupt.String};
            En = {Settings.Pin.En.String Settings.Pin.Light.String};
            Dir = {Settings.Pin.Dir.String};
            if  ImproperPinValues(Terminal,Apin,En,Dir)
                Status(sprintf('Recheck Values'))
            else
                Status(sprintf('Values Check Out'))
                for i = 1:5
                    Pins.Ana(i) = eval(Apin{i});
                    Pins.En(i) = eval(En{i});
                    Pins.Dir(i) = eval(Dir{i});
                end
                Active = Settings.Pin.High.Value;
                Pins.Set = 1;
                Pins.Light = str2double(Settings.Pin.Light.String);
                Pins.Interrupt = str2double(Settings.Pin.Interrupt.String);
                IRQ = strcat('A',Settings.Pin.Interrupt.String);
                configurePin(Arduino,IRQ,'Pullup');
                CalSettings.SelectedObject = Settings.Cal.Button(1);
                CalButton_Callback(Settings.Cal.Button(1));
                CalPanel.Visible = 'on';
                Stop;
            end
        else
            Status('Arduino board not Specified');
        end
    end
    
    %Changes the configuration of the robot arm
    function Slide_Callback(source,~)
        %Called by Data.Slide<> in the DisplayTab
        index = source.UserData;
        if strcmp(source.String,'P')
            Data.PowerText(index).String = num2str(source.Value,4);
        else
            Data.SlideText(index).String = num2str(source.Value*RadCheck(index,Data.Slide,Data.Radians),3);
            Update(Data)
        end
    end
    
    %Sets the limit of the chosen slide
    function SlideLimit_Callback(source,~)
        %Called by Settings.Slide.Label in the SlideSettings uipanel
        val = source.UserData;
        if ~isempty(Settings.Slide.Min(val).String)&&~isempty(Settings.Slide.Max(val).String)
            Data.Slide(val).Min = str2double(Settings.Slide.Min(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.Slide(val).Max = str2double(Settings.Slide.Max(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.Slide(val).Value = str2double(Settings.Slide.Max(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.SlideText(val).String = Data.Slide(val).Value*RadCheck(val,Data.Slide,Data.Radians);
            Update(Data) 
        end
    end
    
    %Sets slide values
    function SlideText_Callback(source,~)
        %Called by Data.SlideText<> in the DisplayTab
        index = source.UserData;
        Data.Slide(index).Value = str2double(source.String)/RadCheck(index,Data.Slide,Data.Radians);
        Update(Data)
    end
    
    %EMERGENCY STOP
    function Stop(~,~)
        Interrupt = 1;
        if Pins.Set
            digitalWrite(Arduino,Pins.En,zeros(size(Pins.En)));
            digitalWrite(Arduino,Pins.Dir,zeros(size(Pins.Dir)));
        else
            Status('Pins haven''t been setup yet');
        end
    end
    
    %Allows the status display and remote to span multiple tabs
    function TabChanged_Callback(source,event)
        %Called whenever the user switches tabs
        if strcmp(source.Units,'normalized')
            xmax1 = 280;
            ymax1 = 500;
            xmax2 = 252;
            ymax2 = 200;
        else
            xmax1 = 1;
            xmax2 = xmax1;
            ymax1 = 1;
            ymax2 = ymax1;
        end
        Status();
        switch event.NewValue.Title
            case 'Arduino Setup'
                RemoteSettings.Parent = PinSettings;
                RemoteSettings.Position = [175/xmax2 22/ymax2 26/xmax2 120/ymax2];
                Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
            case 'Display'
                RemoteSettings.Parent = DisplayTab;
                RemoteSettings.Position = [2/xmax1 345/ymax1 26/xmax1 152/ymax1];
                if Settings.Advanced1.Value || Settings.Advanced2.Value || Settings.DH.Value
                    Data.Stop.Position = [662.5/750,12/500,70/750,80/500];
                else
                    Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
                end
            case 'Automation'
                RemoteSettings.Parent = ClosedLoopPanel;
                RemoteSettings.Position = [10/xmax1 253/ymax1 35/xmax1 245/ymax1];
                Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
        end
    end
    
    %Uploads one or more arm configurations to the arduino
    function Upload_Callback(source,~)
        %Called by Data.Upload<> and Data.UploadAll in the DisplayTab
        val = source.UserData;
        goal = [Data.Slide(val).Value];
        if CheckCal(Pins.Pot)
            move(val,goal);
        elseif ~Pins.Set
            Status('Setup pins first');
        else
            Status('Pins need to be calibrated');
        end
    end
    
    %changes the Plot view
    function Display_Callback(source,~)
        %called by all buttons in the DisplaySettings menu in the DisplayTab
        switch source.String
            case 'YX'
                view(-90,90);
            case 'XZ'
                view(0,0);
            case 'XZ'''
                view(-180,0);
            case 'YZ'
                view(90,0);
            case 'YZ'''
                view(-90,0);
            case 'RESET'
                view(-150,30);
            case 'ISO'
                [az,~]=view;
                az = az/45 - mod(az,45)/45;
                if -1 <= az && az < 1
                    view(-45,30);
                elseif 1 <= az && az <3
                    view(45,30);
                elseif 3 <= az && az <5 ||-5< az && az <-3
                    view(135,30);
                else
                    view(225,30);
                end
            case 'ROTATE'
                if source.Value
                    rotate3d on
                else
                    rotate3d off
                end
        end
    end
    
    %Displays the slide limit edit boxes indicated by the menu
    function ViewSlider_Callback(source,~)
        %Called by Settings.Slide.ShowBoxes in the DisplayTab
        val = Settings.Slide.Menu.Value;
        if source.Value
            Settings.Slide.TopLabel.Visible = 'on';
            Settings.Slide.Min(val).Visible = 'on';
            Settings.Slide.Max(val).Visible = 'on';
            Settings.Slide.Label(val).Visible = 'on';
            Settings.Slide.Menu.Enable = 'off';
        else
            Settings.Slide.TopLabel.Visible = 'off';
            Settings.Slide.Min(val).Visible = 'off';
            Settings.Slide.Max(val).Visible = 'off';
            Settings.Slide.Label(val).Visible = 'off';
            Settings.Slide.Menu.Enable = 'on';
        end
    end

%% FUNCTIONS

    %SET AN ARM CONFIGURATION (angle values in degrees)
    function Config(input,varargin)
        clc
        Interrupt = 0;
        if nargin == 1
            for i = 1:numel(input)
                Data.Slide(i).Value = input(i)/RadCheck(i,Data.Slide,Data.Radians);
                Slide_Callback(Data.Slide(i));
            end
        else % Fancy Display for Preview window
            SlideConfig = GetPlotConfig(Data.Slide,Data.Radians);
            for i = 1:5
                n = input(6-i) - SlideConfig(6-i);
                m = n - rem(n,1);
                if m
                    for k = 1:abs(m)
                        Data.Slide(6-i).Value = (SlideConfig(6-i)+k*m/abs(m))/RadCheck(6-i,Data.Slide,Data.Radians);
                        pause(.05);
                        Update(Data);
                        if ~Program.Display.Value
                            break
                        end
                    end
                    if rem(n,1)
                        Data.Slide(6-i).Value = input(6-i)/RadCheck(6-i,Data.Slide,Data.Radians);
                    end
                end
                if ~Program.Display.Value
                    break
                end
                Slide_Callback(Data.Slide(6-i));
            end
        end
    end
    
    %GET DIRECTION TOWARDS GOAL
    function [Direction,Dist] = getDir(val,goal)
        Direction = ones(size(val));
        if isnumeric(goal)            
            range = [Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value];
            scale = range./abs(range); %([Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value])./abs([Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value]); 
            goalVoltage = map(goal',Data.ArmLimits(val,:),[[Pins.Pot.Min(val).Value]' [Pins.Pot.Max(val).Value]'].*scale');  
            goalOffset = (goalVoltage - analogRead(Arduino,(Pins.Ana(val))').*scale');
            Dist = abs(goalOffset);
            Direction(abs(goalOffset) < tolerance) = NaN;
            Direction(abs(goalOffset) > tolerance & goalOffset < 0) = 0;
            Direction(abs(goalOffset) > tolerance & goalOffset > 0) = 1;
        else
            Dist = Direction;
            Direction(strcmp(goal,'-')) = 0;
            Direction(strcmp(goal,'+')) = 1;
            Direction(~(strcmp(goal,'+')|strcmp(goal,'-'))) = NaN;
        end
    end
    
    %MOVE ARM
    function move(val,goal)
        Dir = getDir(val,goal); % Get the direction of the movement (0) for negative, (1) for positive, and NaN for idle joints
        IdlePins = isnan(Dir); % Get index of Idle joints
        digitalWrite(Arduino,Pins.Dir(val(~IdlePins)),Dir(~IdlePins)); %Update the Direction pins of non-idle joints
        Interrupt = 0; % reset interrupt
        if analogRead(Arduino,Pins.Interrupt)> 4 %Check for the shunt and only move with the shunt removed.
            if ~(strcmp(goal,'+')||strcmp(goal,'-')) && sum(~IdlePins) %perform this section is source is not the virtual remote
                %analogWrite(Arduino,Pins.En(val(~IdlePins)),Data.PowerSlide(Val(~IdlePins)).Value/100*2.5*ones(numel(find(~IdlePins)),1));
                digitalWrite(Arduino,Pins.En(val(~IdlePins)),ones(numel(find(~IdlePins)),1)); %Start move by enabling non idle joints
                n = numel(find(IdlePins));
                while (n < numel(Dir) && ~Interrupt && analogRead(Arduino,Pins.Interrupt) > 4)
                    Dir = getDir(val,goal);
                    NewIdlePins = isnan(Dir);
                    n = numel(find(NewIdlePins));
                    NewIdlePins = xor(NewIdlePins,IdlePins);
                    pause(0.1);
                    if sum(NewIdlePins)
                        digitalWrite(Arduino,Pins.En(val(NewIdlePins)),zeros(numel(find(NewIdlePins)),1));
                    end
                    IdlePins = IdlePins|NewIdlePins;
                end
                EmStop(Arduino,Active,Pins.En,(1:5));
            else 
                for i = 1:numel(val)
                    if strcmp(goal(i),'+')||strcmp(goal(i),'-')
                        analogWrite(Arduino,Pins.En(val(i)),Data.PowerSlide(val(i)).Value/100 * 2.5);
                    else
                        EmStop(Arduino,Active,Pins.En,val(i));
                    end
                end
            end
        else
            Status('Remove Jumper or release RESET');
        end
    end

    %Display update
    function Status(message,varargin)
        index = size(Terminal.String,1);
        if nargin == 0
            Terminal.String = '';
        elseif index && nargin < 2
            previous = Terminal.String(index,:);
            if contains(previous,message)
                if index == 1
                    Terminal.String = strcat(Terminal.String,'.');
                else
                    Terminal.String(index,:) = strcat(Terminal.String(index,:),'.');
                end
            elseif index < 7
                Terminal.String = [Terminal.String;{message}];
            else
                Terminal.String(1,:) = [];
                Terminal.String = [Terminal.String;{message}];
            end
        else
            Terminal.String = message;
        end        
    end
    
end
%{
*    analogRead
*    analogWrite
*    CheckCal
*    digitalWrite
*       EmStop
*    GetPlotConfig
*    GetRobotConfig
*    ImproperPinValues
*    map
*    RadCheck
    -stlReadBinary
*    Update
%}
