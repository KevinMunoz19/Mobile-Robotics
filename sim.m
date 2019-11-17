% Debe encapsularse cualquier interacci�n entre MATLAB y el simulador
% dentro de una funci�n, si no los comandos NO funcionar�n adecuadamente
function simulacion()
    %% Conexi�n con el simulador
    % Se inicializa la conexi�n al simulador
    disp('Program started');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % Se verifica si fue posible establecer la conexi�n con el simulador 
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Nos aseguramos que se cierre la conexi�n cuando el script se vea
    % interrumpido
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % Se inicia la simulaci�n
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
    
    %% Obtenci�n de handles
    % Se define un struct que contendr� todos los handles de los objetos
    % en la escena de V-REP
    h = struct('id', id); 
    
    % Se obtiene el handle del sensor de vision
    [~, h.cam] = vrep.simxGetObjectHandle(id, 'Vision_sensor', vrep.simx_opmode_oneshot_wait);
    
    % Se obtienen los handles de los dummies (metas)
    goals = -ones(1, 4);
    for i = 0:3
        [~, goals(i+1)] = vrep.simxGetObjectHandle(id, sprintf('meta%d', i), vrep.simx_opmode_oneshot_wait);
    end
    h.goals = goals
    
    % Se obtiene el handle del chasis del robot (para encontrar su
    % posici�n)
    [~, h.robotchasis] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_visible', vrep.simx_opmode_oneshot_wait); 
    
    % Se obtienen los handles de los motores del robot
    [~, h.leftmotor] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait);
    [~, h.rightmotor] = vrep.simxGetObjectHandle(id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait);
    
    %% Inicializaci�n 
    % Nos aseguramos que todo est� establecido antes de comenzar (se espera
    % que empiece la simulaci�n) 
    pause(0.2);
    
    % Se define (en MATLAB) el tiempo de simulaci�n empleado por V-Rep 
    % (garantizar que el c�digo corra cercano a este valor) 
    timestep = 0.05;

    % Nos aseguramos que todo est� establecido antes de comenzar 
    pause(2);
    
    %% Simulaci�n en "tiempo real"
    dt = timestep;
    
    % Par�metros de la secuencia de comandos desde MATLAB
    t0 = 0; % Tiempo inicial
    tf = 10; % Tiempo final
    k = 0; 
    K = (tf-t0)/dt; % N�mero total de iteraciones 
    
    % =====================================================================
    % MODIFICAR AQU�
    % =====================================================================
    % Obtenga la posici�n xy de los dummies (metas) dispersos dentro de la
    % escena de V-REP y gu�rdelos dentro de un �nico array (4x2) llamado
    % goals_pos. Adicionalmente, obtenga la posici�n xy inicial del robot y
    % col�quela dentro de un vector (1x2) llamado pos0.
    % ---------------------------------------------------------------------
    goals_pos = zeros(4,2);
    pos0 = zeros(1,2);
   
    [~,d0]=vrep.simxGetObjectPosition(id,h.goals(1),-1,vrep.simx_opmode_oneshot_wait);
    [~,d1]=vrep.simxGetObjectPosition(id,h.goals(2),-1,vrep.simx_opmode_oneshot_wait);
    [~,d2]=vrep.simxGetObjectPosition(id,h.goals(3),-1,vrep.simx_opmode_oneshot_wait);
    [~,d3]=vrep.simxGetObjectPosition(id,h.goals(4),-1,vrep.simx_opmode_oneshot_wait);
    %metiendo las posiciones
    goals_pos(1,1)=d0(1);
    goals_pos(1,2)=d0(2);
    goals_pos(2,1)=d1(1);
    goals_pos(2,2)=d1(2);
    goals_pos(3,1)=d2(1);
    goals_pos(3,2)=d2(2);
    goals_pos(4,1)=d3(1);
    goals_pos(4,2)=d3(2)
        
    %posicion del robot
    [~,robotPos]=vrep.simxGetObjectPosition(id,h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
    pos0(1,1)=robotPos(1);
    pos0(1,2)=robotPos(2);
    
    %orientacionRobot
    [~,OrientationR]=vrep.simxGetObjectOrientation(id,h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
    theta=OrientationR(1,3);

    % =====================================================================
    
    % =====================================================================
    % MODIFICAR AQU� (Hasta en la segunda parte)
    % =====================================================================
    % Obtenga una imagen I del mapa empleando el sensor de visi�n dentro de
    % la escena, aplique thresholding para convertirlo en una occupancy
    % grid map y emplee las funciones de planificaci�n de la Robotics 
    % Toolbox (consultar el documento Navigation-PC.pdf) para encontar un 
    % recorrido path que vaya desde la posici�n inicial del robot hasta la 
    % meta deseada.
    % ---------------------------------------------------------------------
    [~,~,matrix_image]=vrep.simxGetVisionSensorImage2(id,h.cam,1,vrep.simx_opmode_oneshot_wait);
    I = matrix_image;
    map = I>230;
    map=flip(map,2);
    map=double(map);
    imshow(map)
    radius_in_cells=6;
    
    d1_m=(d3(1) + 5) * (256 - 1) / (5 +5) + 1;
    d2_m=(d3(2) + 5) * (256 - 1) / (5 +5) + 1;
    
  
    st1_m=(robotPos(1) +5) * (256 - 1) / (5 +5) + 1;
    st2_m=(robotPos(2) +5) * (256 - 1) / (5 +5) + 1;
    
    goal=[round(d2_m);round(d1_m)];
    start=[round(st1_m);round(st2_m)];
    
    Dst=Dstar(map,'inflate',radius_in_cells);        
    Dst.plan(goal);                                                           
    path = Dst.query(start,'animate');
    
    % =====================================================================
    
    % El ciclo se repite siempre y cuando no se haya finalizado con la
    % secuencia de comandos desde MATLAB. Puede variarse el par�metro tf 
    % (en segundos) para hacer que la simulaci�n completa sea m�s larga o
    % corta
    while(k < K)
        tic; 
        % =================================================================
        % MODIFICAR AQUI
        % =================================================================
        % Encuentre la posici�n xy y orientaci�n theta actual del robot y 
        % almac�nela en las variables x, y & theta respectivamente.
        % Recuerde que theta debe estar en radianes (verifique qu� retorna
        % la funci�n de V-REP)
        % -----------------------------------------------------------------
        [~,robotPos]=vrep.simxGetObjectPosition(id,h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
        pos0(1,1)=robotPos(1);
        pos0(1,2)=robotPos(2);
        
        [~,OrientationR]=vrep.simxGetObjectOrientation(id,h.robotchasis,-1,vrep.simx_opmode_oneshot_wait);
        theta=OrientationR(1,3);
        
        x = pos0(1,1);
        y = pos0(1,2);
        theta = theta;
        % =================================================================
        
        
        % =================================================================
        % MODIFICAR AQUI
        % =================================================================
        % Defina la posici�n del punto (xg,yg) a seguir por el robot m�vil
        % (para el caso de control punto a punto, (xg,yg) es igual a la
        % posici�n de la meta0 mientras que va cambiando para cuando desee
        % seguirse un recorrido -hasta en la tercera parte-). Ayuda: para 
        % el caso de recorridos, puede cambiar el valor de este punto 
        % cuando la distancia  norm([xg-x; yg-y]) sea lo suficientemente 
        % peque�a, asumiendo que el punto (x,y) corresponde a la posici�n 
        % actual del robot.
        % -----------------------------------------------------------------
        xg = goals_pos(1,1);
        yg = goals_pos(1,2);
        % =================================================================
        
        % =================================================================
        % MODIFICAR AQUI
        % =================================================================
        % Actualice las velocidades phi_l y phi_r para los motores del 
        % robot m�vil empleando el algoritmo de control visto en clase. 
        % Modifique las constantes Kp y Ko hasta obtener un comportamiento 
        % razonable.
        % -----------------------------------------------------------------
        Kp=0.9;
        Ko=0.9;
        l=0.1905;
        r=0.0975;
        %formulazo
        Cp=[xg;yg]-[x;y];
        
        tetha_g=atan2((yg-y),(xg-x));
        Co=atan2((sin(tetha_g-theta)),(cos(tetha_g-theta)));
        
        v=Kp*Cp;
        w=Ko*Co;
        
        phi_r = (v+w*l)/r;
        phi_l = (v-w*l)/r;
        % =================================================================
        
        % Se actualizan las velocidades de los motores del robot
        vrep.simxPauseCommunication(id, true);
        vrep.simxSetJointTargetVelocity(id, h.rightmotor, phi_r, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(id, h.leftmotor, phi_l, vrep.simx_opmode_oneshot);
        vrep.simxPauseCommunication(id, false);
        
        k = k + 1;
        
        % Nos aseguramos de no ir m�s r�pido que el simulador (cada 
        % iteraci�n debe tomar 50 ms)
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end
    %disp(x);disp(xg);disp(y);disp(yg);
    disp(Cp);
    pause(2);
    
    %% Finalizaci�n
    % Se termina la comunicaci�n entre MATLAB y V-Rep
    vrep.delete();
end