function varargout = EJEMPLO2(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @EJEMPLO2_OpeningFcn, ...
                   'gui_OutputFcn',  @EJEMPLO2_OutputFcn, ...
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

% --- Executes just before EJEMPLO2 is made visible.
function EJEMPLO2_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for EJEMPLO2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = EJEMPLO2_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
  global regreso
  regreso = 1;

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
  global moi
  moi = 1;


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

pi = 3.141592653589793;
r = 0.097;
Longitud = 0.331;
VT = 0.05;
Wr = VT / r;
Wl = VT / r;
Yd = 0;
Xd = 0;
phi_deseado = 0;
error = 0;
Sensor_1 = 0;
Sensor_2 = 0;
Sensor_3 = 0;
Sensor_4 = 0;
Sensor_5 = 0;
Sensor_6 = 0;
Sensor_7 = 0;
Sensor_8 = 0;
AUX = 0;
AUX_1 = 0;
objeto_detectado = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
contador = 1;
valor = 0;
Temperatura_inferior = 100;
Temperatura_superior = 0;
Humedad_inferior = 100;
Humedad_superior = 0;
global moi
moi = 0;
global regreso
regreso = 0;

 
if (clientID ~=-1)  % Si la conexion es exitosa entra en el bucle
 
    set(handles.pushbutton5,'Enable','off')  

    % Crear un handle para los motores
    [err, motor_izquierdo] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait);
    [err, motor_derecho]   = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait);

    % Crear un handle para la posicion del movil    
    [retVal, quad] = vrep.simxGetObjectHandle (clientID, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait);

    %Crear un handle para los sensores de distancia
    [err, sensor_1] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', vrep.simx_opmode_oneshot_wait);
    [err, sensor_2] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor2', vrep.simx_opmode_oneshot_wait);
    [err, sensor_3] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor3', vrep.simx_opmode_oneshot_wait);
    [err, sensor_4] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor4', vrep.simx_opmode_oneshot_wait);
    [err, sensor_5] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_oneshot_wait);
    [err, sensor_6] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor6', vrep.simx_opmode_oneshot_wait);
    [err, sensor_7] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor7', vrep.simx_opmode_oneshot_wait);
    [err, sensor_8] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8', vrep.simx_opmode_oneshot_wait);

    [err, camara] = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait);

    % Se le da una velocidad de inicio
    vrep.simxSetJointTargetVelocity(clientID, motor_izquierdo, Wl, vrep.simx_opmode_streaming);
    vrep.simxSetJointTargetVelocity(clientID, motor_derecho, -Wr, vrep.simx_opmode_streaming);
    pause(0.1);

    % Se pide la pasicion del movil en lso ejes X y Y
   [retVal, posXY] = vrep.simxGetObjectPosition (clientID, quad, -1, vrep.simx_opmode_streaming);

    Xinicial = posXY(1);  % Coordenada X inicial
    Yinicial = posXY(2);  % Coordenada Y inicial
    
    set(handles.pushbutton4,'Enable','on')
    set(handles.pushbutton6,'Enable','off')
    set(handles.pushbutton7,'Enable','off')
    set(handles.pushbutton8,'Enable','off')

    while (vrep.simxGetConnectionId(clientID)~=-1)  % Mientras la simulacion este activa hay que correr el bucle
          
          if (moi == 1)
             vrep.simxFinish(clientID);  % Finaliza la conexion
             set(handles.pushbutton5,'Enable','on')
             set(handles.pushbutton4,'Enable','off')
             set(handles.pushbutton6,'Enable','on')
             set(handles.pushbutton7,'Enable','on')
             set(handles.pushbutton8,'Enable','on')
          end

          % Se pide la pasicion del movil en lso ejes X y Y
          [retVal, posXY] = vrep.simxGetObjectPosition (clientID, quad, -1, vrep.simx_opmode_streaming);
          % Se le pide la orientacion del movil
          [retVal, Orientacion] = vrep.simxGetObjectOrientation(clientID, quad, -1, vrep.simx_opmode_streaming);

          X = posXY(1);  % Coordenada X
          Y = posXY(2);  % Coordenada Y

          axes(handles.axes1)
          plot(X,Y,'*','Color',[0, 0, 0])  % Para marcar la localicacion del movil en los ejes X y Y
              hold on        % Para trazar la pendiente que recorre el movil en el grafico
          title('MAPA 2D')
          xlabel('Coordenada en X')
          ylabel('Coordenada en Y')

          phi_actual_rad = Orientacion(3);  % Angulo actual del movil
          phi_actual_gra = (phi_actual_rad * 180)/ pi;

          if ( (Sensor_3 == 0) && (Sensor_4 == 0) && (Sensor_5 == 0) && (Sensor_6 == 0) )
             AUX_1 = 0;
          end

          if ( (AUX_1 == 0) && (regreso == 0) )
             phi_deseado = phi_actual_rad; % Angulo deseando en rad para exploracion      
          end

          if (regreso == 1)
             phi_deseado = atan2(Yinicial-Y, Xinicial-X); % Angulo deseando en rad desde la posicion inicial para llegar al punto deseado 
          end

          error = phi_deseado - phi_actual_rad;  % Error entre el angulo deseado y el angulo actual del movil

          if ( (error <= 0.05) && (error >= -0.05) )
              VT = 0.1;
              AUX = 0;
          else
              VT = 0.05;
          end

          Vr = VT + (error * Longitud / 2);  % Nueva velocidad lineal de la rueda derecha 
          Vl = VT - (error * Longitud / 2);  % Nueva velocidad lineal de la rueda izquierda
          Wr = Vr / r;  % Velocidad rotativa de la rueda derecha
	      Wl = Vl / r;  % Velocidad rotativa de la rueda izquierda

          % Se aplica la velocidad rotativa calculada a las ruedas para corregir el error en los angulos        
          vrep.simxSetJointTargetVelocity(clientID, motor_izquierdo, Wl, vrep.simx_opmode_streaming);
          vrep.simxSetJointTargetVelocity(clientID, motor_derecho, Wr, vrep.simx_opmode_streaming);

          %Leer los dos sensores. En este ejemplo solo nos interesa la variable 'estado'.                 
          [err, estado_sensor_1, punto_1, objeto_detectado_1, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_1, vrep.simx_opmode_streaming);
          [err, estado_sensor_2, punto_2, objeto_detectado_2, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_2, vrep.simx_opmode_streaming);
          [err, estado_sensor_3, punto_3, objeto_detectado_3, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_3, vrep.simx_opmode_streaming);
          [err, estado_sensor_4, punto_4, objeto_detectado_4, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_4, vrep.simx_opmode_streaming);
          [err, estado_sensor_5, punto_5, objeto_detectado_5, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_5, vrep.simx_opmode_streaming);
          [err, estado_sensor_6, punto_6, objeto_detectado_6, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_6, vrep.simx_opmode_streaming);
          [err, estado_sensor_7, punto_7, objeto_detectado_7, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_7, vrep.simx_opmode_streaming);
          [err, estado_sensor_8, punto_8, objeto_detectado_8, vector_normal] = vrep.simxReadProximitySensor(clientID, sensor_8, vrep.simx_opmode_streaming);
 
          %Para mostrar las lecturas de los sensores, creamos un array con el nombre de cada sensor y su estado.
          Sensor_1 = estado_sensor_1;
          Sensor_2 = estado_sensor_2;
          Sensor_3 = estado_sensor_3;
          Sensor_4 = estado_sensor_4;
          Sensor_5 = estado_sensor_5;
          Sensor_6 = estado_sensor_6;
          Sensor_7 = estado_sensor_7;
          Sensor_8 = estado_sensor_8;


          if (Sensor_4 == 1)  
              for i=1:28
                  if (objeto_detectado_4 == objeto_detectado(i) ) 
                      valor = 1;
                  end
              end
              if (valor == 0)        
                 objeto_detectado(contador) = objeto_detectado_4;
                 contador = contador + 1;   
                 Xg = 0.5 * (cos(phi_actual_gra)) + X;
                 Yg = 0.5 * (sin(phi_actual_gra)) + Y;
                 axes(handles.axes1)
                 plot(Xg, Yg, '*', 'Color',[1, 1, 0], 'LineWidth', 10)
                 hold on   
              end
              valor = 0;
          end

          if (Sensor_5 == 1)  
              for i=1:28
                  if (objeto_detectado_5 == objeto_detectado(i) ) 
                      valor = 1;
                  end
              end
              if (valor == 0)        
                 objeto_detectado(contador) = objeto_detectado_5;
                 contador = contador + 1;                 
                 Xg = 0.5 * (cos(phi_actual_gra)) + X;
                 Yg = 0.5 * (sin(phi_actual_gra)) + Y;
                 axes(handles.axes1)
                 plot(Xg, Yg, '*', 'Color',[1, 1, 0], 'LineWidth', 10)                
                 hold on   
              end
              valor = 0;
          end

       

          if ( (Sensor_4 == 1) && (Sensor_5 == 1) )
              if ( (Sensor_3 == 0) && (Sensor_2 == 0) )
                  if (AUX == 0)
                      phi_deseado = phi_actual_rad + 0.785398;
                      AUX = 1;
                      AUX_1 = 1;
                  end
              elseif ( (Sensor_6 == 0) && (Sensor_7 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 0.785398;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_2 == 0) && (Sensor_1 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_7 == 0) && (Sensor_8 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif (Sensor_1 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif (Sensor_8 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              end
          end


          if ( (Sensor_4 == 1) && (Sensor_5 == 0) )
              if (Sensor_6 == 0) 
                  if (AUX == 0)
                      phi_deseado = phi_actual_rad - 0.401426;
                      AUX = 1;
                      AUX_1 = 1;
                  end
              elseif ( (Sensor_3 == 0) && (Sensor_2 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 0.785398;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_7 == 0) && (Sensor_8 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_2 == 0) && (Sensor_1 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif (Sensor_8 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end         
              elseif (Sensor_1 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              end
          end

         
          if ( (Sensor_5 == 1) && (Sensor_4 == 0) )
              if (Sensor_3 == 0) 
                  if (AUX == 0)
                      phi_deseado = phi_actual_rad + 0.401426;
                      AUX = 1;
                      AUX_1 = 1;
                  end
              elseif ( (Sensor_6 == 0) && (Sensor_7 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 0.785398;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_2 == 0) && (Sensor_1 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif ( (Sensor_7 == 0) && (Sensor_8 == 0) )
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.16937;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              elseif (Sensor_1 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad + 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end         
              elseif (Sensor_8 == 0) 
                      if (AUX == 0)
                          phi_deseado = phi_actual_rad - 1.5708;
                          AUX = 1;
                          AUX_1 = 1;
                      end
              end
          end

          if ( (Sensor_3 == 1) && (Sensor_4 == 0) && (Sensor_5 == 0) )
              if (AUX == 0)
                  phi_deseado = phi_actual_rad - 0.401426;
                  AUX = 1;
                  AUX_1 = 1;
              end
          end

          if ( (Sensor_6 == 1) && (Sensor_4 == 0) && (Sensor_5 == 0) )
              if (AUX == 0)
                  phi_deseado = phi_actual_rad + 0.401426;
                  AUX = 1;
                  AUX_1 = 1;
              end
          end

          %Capturar imagen con la camara
          [errorCode,resolution,img]=vrep.simxGetVisionSensorImage2(clientID,camara,0,vrep.simx_opmode_oneshot_wait);
         
          %Mostrar imagen en una ventana
          axes(handles.axes2);
          imshow(img);
          drawnow;

          Temperatura_actual = randi([15 80],[1 1]);  % Obtener un valor aleatorio entre 15 y 80
          Temperatura_string = num2str(Temperatura_actual);
          set(handles.edit2,'string',Temperatura_string);

          if (Temperatura_actual > Temperatura_superior)
             Temperatura_superior = Temperatura_actual;
             X_temp_superior = X;
             Y_temp_superior = Y;
          end

          if (Temperatura_actual < Temperatura_inferior)
             Temperatura_inferior = Temperatura_actual;
             X_temp_inferior = X;
             Y_temp_inferior = Y;
          end

          Humedad_actual = randi([0 100],[1 1]);  % Obtener un valor aleatorio entre 0 y 100
          Humedad_string = num2str(Humedad_actual);
          set(handles.edit3,'string',Humedad_string);

          if (Humedad_actual > Humedad_superior)
             Humedad_superior = Humedad_actual;
             X_hum_superior = X;
             Y_hum_superior = Y;
          end

          if (Humedad_actual < Humedad_inferior)
             Humedad_inferior = Humedad_actual;
             X_hum_inferior = X;
             Y_hum_inferior = Y;
          end

          CH4_actual = randi([0 100],[1 1]);  % Obtener un valor aleatorio entre 0 y 100
          CH4_string = num2str(CH4_actual);
          set(handles.edit8,'string',CH4_string);
          if (CH4_actual > 98)
             axes(handles.axes3)
             plot(X,Y,'O','Color',[1, 0, 0])  % Para marcar un punto ROJO donde hay deteccion de CH4
             hold on        
          end

          CO_actual = randi([0 100],[1 1]);  % Obtener un valor aleatorio entre 0 y 100
          CO_string = num2str(CO_actual);
          set(handles.edit6,'string',CO_string);
          if (CO_actual > 98)
             axes(handles.axes3)
             plot(X,Y,'*','Color',[0, 1, 0])  % Para marcar un asterisco VERDE donde hay deteccion de CO
             hold on        
          end

          CO2_actual = randi([0 100],[1 1]);  % Obtener un valor aleatorio entre 0 y 100
          CO2_string = num2str(CO2_actual);
          set(handles.edit7,'string',CO2_string);
          if (CO2_actual > 98)
             axes(handles.axes3)
             plot(X,Y,'+','Color',[0, 0, 1])  % Para marcar un + AZUL donde hay deteccion de CO2
             hold on        
          end

          H2SO4_actual = randi([0 100],[1 1]);  % Obtener un valor aleatorio entre 0 y 100
          H2SO4_string = num2str(H2SO4_actual);
          set(handles.edit9,'string',H2SO4_string);
          if (H2SO4_actual > 98)
             axes(handles.axes3)
             plot(X,Y,'x','Color',[0, 0, 0])  % Para marcar una x NEGRO donde hay deteccion de H2SO4
             hold on        
          end
       
 
          pause(0.1);  % Pausa de 0.5 segundos  
    end
end
vrep.simxFinish(clientID);  % Finaliza la conexion
set(handles.pushbutton5,'Enable','on')
set(handles.pushbutton4,'Enable','off')
set(handles.pushbutton6,'Enable','on')
set(handles.pushbutton7,'Enable','on')
set(handles.pushbutton8,'Enable','on')


function edit2_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)

function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit6_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit8_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit9_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
try
formatos = {'*.jpg','JPEG (*.jpg)';'*.tif','TIFF (*.tif)';'*.epsc','EPS (*.eps)';'*.pdf','PDF (*.pdf)'};
[nomb,ruta] = uiputfile(formatos,'GUARDAR IMAGEN');
 
if or(and(isempty(nomb),isempty(nomb)),nomb==0)
    msgbox('No indico un nombre para guardar archivo', 'Aviso','warn');
else
figura=figure(2);
set(figura,'InvertHardcopy','off','Color','white');
objeto_1 = copyobj(handles.axes1,figura);
set(objeto_1,'Units','normalized','Position',[0.05 0.1 0.90 0.80])
set(figura, 'PaperPositionMode', 'auto')

saveas(gcf,[ruta nomb])
close(figura)
end
 
catch
 
 msgbox('No guardo el archivo', 'Aviso','warn');
 
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
try
formatos = {'*.jpg','JPEG (*.jpg)';'*.tif','TIFF (*.tif)';'*.epsc','EPS (*.eps)';'*.pdf','PDF (*.pdf)'};
[nomb,ruta] = uiputfile(formatos,'GUARDAR IMAGEN');
 
if or(and(isempty(nomb),isempty(nomb)),nomb==0)
    msgbox('No indico un nombre para guardar archivo', 'Aviso','warn');
else
figura=figure(2);
set(figura,'InvertHardcopy','off','Color','white');
objeto_1 = copyobj(handles.axes3,figura);
set(objeto_1,'Units','normalized','Position',[0.05 0.1 0.90 0.80])
set(figura, 'PaperPositionMode', 'auto')

saveas(gcf,[ruta nomb])
close(figura)
end
 
catch
 
 msgbox('No guardo el archivo', 'Aviso','warn');
 
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
cla(handles.axes1,'reset')
cla(handles.axes3,'reset')
set(handles.pushbutton6,'Enable','off')
set(handles.pushbutton8,'Enable','off')
