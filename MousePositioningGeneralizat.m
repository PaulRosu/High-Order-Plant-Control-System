function varargout = MousePositioningGeneralizat(varargin)
    % MOUSEPOSITIONINGGENERALIZAT MATLAB code for MousePositioningGeneralizat.fig
    %      MOUSEPOSITIONINGGENERALIZAT, by itself, creates a new MOUSEPOSITIONINGGENERALIZAT or raises the existing
    %      singleton*.
    %
    %      H = MOUSEPOSITIONINGGENERALIZAT returns the handle to a new MOUSEPOSITIONINGGENERALIZAT or the handle to
    %      the existing singleton*.
    %
    %      MOUSEPOSITIONINGGENERALIZAT('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in MOUSEPOSITIONINGGENERALIZAT.M with the given input arguments.
    %
    %      MOUSEPOSITIONINGGENERALIZAT('Property','Value',...) creates a new MOUSEPOSITIONINGGENERALIZAT or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before MousePositioningGeneralizat_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to MousePositioningGeneralizat_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES
    
    % Edit the above text to modify the response to help MousePositioningGeneralizat
    
    % Last Modified by GUIDE v2.5 25-Mar-2020 16:48:01
    
    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @MousePositioningGeneralizat_OpeningFcn, ...
                       'gui_OutputFcn',  @MousePositioningGeneralizat_OutputFcn, ...
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
    
    
    % --- Executes just before MousePositioningGeneralizat is made visible.
    function MousePositioningGeneralizat_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to MousePositioningGeneralizat (see VARARGIN)
    
    % Choose default command line output for MousePositioningGeneralizat
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    
    % init screen size
    global xmax;
    global ymax;
    global stopflag;
    
    screensize = get(0,'ScreenSize');
    xmax = screensize(3);
    ymax = screensize(4);
    stopflag = true;
    
    level = get(handles.LevelSlider,'Value');
    set(handles.Level,'String',num2str(level-1));
    
    set(handles.StartButton,'Visible','on');
    set(handles.StopButton ,'Visible','off');
    
    axes(handles.DisplayAxes);
    plot(3*xmax/4,ymax/2,'ko','MarkerSize',20)
    text(15,ymax-20,'ECRAN','FontName','Times New Roman','FontSize',14,'Color', [0.5,0.5,0.5]);
    hold on
    plot(xmax/4,ymax/2,'ko','MarkerFaceColor','m','MarkerSize',16)
    hold off
    axis([1,xmax,1,ymax]);

    initializeTCPServer();
    
    % UIWAIT makes MousePositioningGeneralizat wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
    
    
    % --- Outputs from this function are returned to the command line.
    function varargout = MousePositioningGeneralizat_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Get default command line output from handles structure
    varargout{1} = handles.output;
    
    % --- Initializes the TCP/IP server using tcpserver
    function initializeTCPServer()
        global tcpServer;
        port = 30000;
        
        % Create a TCP/IP server
        tcpServer = tcpserver("0.0.0.0", port, ...
            "ConnectionChangedFcn", @connectionChanged, ...
            "Timeout", 1);
        
        disp(['TCP/IP Server started on port ', num2str(port)]);

    
    % --- Callback function for connection changes
    function connectionChanged(src, event)
        if src.Connected
            disp('A client has connected to the TCP/IP server.');
        else
            disp('All clients have disconnected from the TCP/IP server.');
        end

    
    % --- Executes on button press in StartButton.
    function StartButton_Callback(hObject, eventdata, handles)
    % hObject    handle to StartButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    global xmax;
    global ymax;
    global stopflag;
    global tcpServer;
    
    set(handles.StartButton,'Visible','off');
    set(handles.StopButton ,'Visible','on');
    stopflag = false;
    
    
    % params
    coeff = [1,1000,100,10,1,1,1,1,1,1,1];
    % initial position
    PL = get(0,'PointerLocation');
    vectorx = [PL(1),zeros(1,10)];
    vectory = [PL(2),zeros(1,10)];
    % final position
    xfin = xmax*unifrnd(0.3,0.7);
    yfin = ymax*unifrnd(0.3,0.7);
    
    historyx = PL(1);
    historyy = PL(2);
    
    TimeLim = 2.5; %s
    thresh = 5; % pix
    Ts = 0.1;
    counter = 0;
    ttime = 0;
    Level = get(handles.LevelSlider,'Value');
    axes(handles.DisplayAxes);
    while(counter < TimeLim && stopflag == false) % keep the dot there for 2.5 second
        
        PL = get(0,'PointerLocation');
        
        if(Level == 1)
                x = PL(1);
                y = PL(2);
        else
            vectorx(Level) = (PL(1)-xmax/2) * coeff(Level) / (xmax/2);
            vectory(Level) = (PL(2)-ymax/2) * coeff(Level) / (ymax/2);    
            for k = Level-1:-1:1
                vectorx(k) = vectorx(k)+Ts*vectorx(k+1);
                vectory(k) = vectory(k)+Ts*vectory(k+1);
            end
            x = vectorx(1);
            y = vectory(1);
        end
        
        x = max(11,x); x = min(xmax-10,x);
        y = max(11,y); y = min(ymax-10,y);
        
        if(x~=vectorx(1))
            vectorx = zeros(size(vectorx));
            vectorx(1) = x;
        end
        if(y~=vectory(1))
            vectory = zeros(size(vectory));
            vectory(1) = y;
        end
        
        historyx = [historyx,x];
        historyy = [historyy,y];

        % Prepare data to send
        dataStruct.xfin = xfin;
        dataStruct.yfin = yfin;
        dataStruct.x = x;
        dataStruct.y = y;
        dataStruct.vectorx = vectorx;
        dataStruct.vectory = vectory;
        dataStruct.Level = Level;
        dataStruct.Ts = Ts;
        dataStruct.ttime = ttime;
        dataStruct.stopflag = stopflag;
        
        % Convert to JSON
        jsonStr = jsonencode(dataStruct);
        jsonBytes = uint8(jsonStr);  % Convert to bytes for transmission
        
        
        % Send JSON data asynchronously if client is connected
        if tcpServer.Connected
            try
                write(tcpServer, jsonBytes, "uint8");
                flush(tcpServer);
            catch ME
                disp('Error sending data asynchronously over TCP/IP:');
                disp(ME.message);
            end
        end
        
        % Display position
        plot(xfin,yfin,'ko','MarkerSize',20)
        hold on
        plot(xmax/2,ymax/2,'k+');
        plot(x,y,'ko','MarkerFaceColor',[TimeLim-counter,counter,0]/(2*TimeLim)+0.5,'MarkerSize',16)
        plot(PL(1),PL(2),'b.');
        hold off
        text(15,ymax-35,'ECRAN','FontName','Times New Roman','FontSize',14,'Color', [0.5,0.5,0.5]);
        text(xmax/2,ymax-35,[num2str(ttime),' s'],'FontName','Times New Roman','FontSize',14,'Color', [0.5,0.5,0.5]);
        axis([1,xmax,1,ymax]);
        pause(Ts)
        
        ttime = ttime + Ts;
        if( sqrt( (x-xfin)^2 + (y-yfin)^2 ) < thresh )
            counter =  counter + Ts;
        else
            counter = 0;
        end
    end
    if(stopflag == true)
        if( sqrt( (x-xfin)^2 + (y-yfin)^2 ) < thresh )
            displaycolour = 'c';
        else
            displaycolour = 'm';
        end
        
        axes(handles.DisplayAxes);
        plot(xfin,yfin,'ko','MarkerSize',20)
        text(15,ymax-35,'ECRAN','FontName','Times New Roman','FontSize',14,'Color', [0.5,0.5,0.5]);
        hold on
        plot(x,y,'ko','MarkerFaceColor',displaycolour,'MarkerSize',16)
        hold off
        axis([1,xmax,1,ymax]);
    end
    if( counter >= TimeLim )
        axes(handles.DisplayAxes);
        hold on
        plot(historyx,historyy,':','color',[0.5,0.5,0.5])
        plot(x,y,'ko','MarkerFaceColor','c','MarkerSize',16)
        hold off
        set(handles.StartButton,'Visible','on');
        set(handles.StopButton ,'Visible','off');
        stopflag = true;
        uiwait(msgbox({['Nivel terminat in ............................. ', num2str(ttime-Ts),' secunde!'];'   '} ,'Bravo!','modal'));
    end

    % Close TCP/IP server
    clear tcpServer;
    disp('TCP/IP Server closed.');
    
    
    
    % --- Executes on button press in StopButton.
    function StopButton_Callback(hObject, eventdata, handles)
    % hObject    handle to StopButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    set(handles.StartButton,'Visible','on');
    set(handles.StopButton ,'Visible','off');
    global stopflag;
    stopflag = true;
    
    
    % --- Executes on button press in HelpButton.
    function HelpButton_Callback(hObject, eventdata, handles)
    % hObject    handle to HelpButton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %path = matlab.desktop.editor.getActiveFilename
    I = imread('Help.png');
    figure, imshow(I)
    
    
    % --- Executes on slider movement.
    function LevelSlider_Callback(hObject, eventdata, handles)
    % hObject    handle to LevelSlider (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Hints: get(hObject,'Value') returns position of slider
    %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    
    global level;
    level = get(hObject,'Value');
    set(handles.Level,'String',num2str(level-1));
    
    
    % --- Executes during object creation, after setting all properties.
    function LevelSlider_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to LevelSlider (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end
    
% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
    % Close TCP/IP server if it's open
    global tcpServer;
    if ~isempty(tcpServer) && tcpServer.IsOpen
        clear tcpServer;
        disp('TCP/IP Server closed on figure deletion.');
    end