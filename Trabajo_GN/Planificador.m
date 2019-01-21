close all;
clearvars -except grafo home hab_1 hab_2 hab_3 Xk
clc;

%% Estimación de la posición inicial
if (exist('Xk')==0)

    % Inicializamos la posición inicial y su covarianza
    global Xk
    xini = 0.5;
    yini = -0.5;
    thetaini = 0;
    Xk = [xini; yini; thetaini];

    % Varianza en la posición
    global Pk
    Pxini = 0.01;
    Pyini = 0.01;
    Pthetaini = 0.01;
    Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];
end

%% Carga del mapa y variables utiles
celdillas = imread('Ocupacion.png');
tam = size(celdillas);
relacion = 21/tam(1);
k = 300;
img = cat(3,celdillas,celdillas,celdillas);
img = img*255;
zonas_c = [255 290 305 370;
           615 705 21 50;
           615 710 520 554;
           615 710 800 840;
           540 615 610 640
           565 638 240 340];

       
%% Muestreo y generación de nodos

if (exist('grafo')==0)

    % Nodos en Cfree
    i = 1;
    v_vacio = zeros(1,20);
    v_inf = Inf(1,20);

    while i<=k

        a = 1 + round(tam(1)*rand);
        b = 1 + round(tam(2)*rand);

        if ((a<=tam(1))&&(b<=tam(2)))
            if (celdillas(a,b)==1)
                grafo(i) = Vertice(i,v_vacio,v_inf,[b a]);
                img(a,b,1) = 0;
                img(a,b,2) = 255;
                img(a,b,3) = 0;
                i=i+1;
            end
        end

    end

    % Nodos en las puertas y zonas criticas
    for j=1:size(zonas_c,1)

        k = k + 10;
        while i<=k

            b = round(zonas_c(j,1) + (zonas_c(j,2)-zonas_c(j,1))*rand);
            a = round(zonas_c(j,3) + (zonas_c(j,4)-zonas_c(j,3))*rand);

            if (celdillas(a,b) == 1)
                grafo(i) = Vertice(i,v_vacio,v_inf,[b a]);
                img(a,b,1) = 0;
                img(a,b,2) = 255;
                img(a,b,3) = 0;
                i=i+1;
            end

       end
    end


    %%%%%%%%% Generación de aristas libres de colisión

    grafo = Generador_Aristas(grafo, celdillas, 0);


    %%%%%%%%% Incluimos los nodos inicial y final

    % Nodo inicial
    nodo_fuente = i;
    home = i;
    pos_ini = [30 30];  % [x,y]
    grafo(nodo_fuente) = Vertice(nodo_fuente,v_vacio,v_inf,pos_ini);
    img(pos_ini(2),pos_ini(1),1) = 0;
    img(pos_ini(2),pos_ini(1),2) = 0;
    img(pos_ini(2),pos_ini(1),3) = 255;

    % Añado los nodos de las habitaciones
    % Habitación 1
    i = i + 1;
    hab_1 = i;
    grafo(i) = Vertice(i,v_vacio,v_inf,[880 180]);

    % Habitación 2
    i = i + 1;
    hab_2 = i;
    grafo(i) = Vertice(i,v_vacio,v_inf,[880 470]);

    % Habitación 3
    i = i + 1;
    hab_3 = i;
    grafo(i) = Vertice(i,v_vacio,v_inf,[880 760]);

    %%%%%%%%% Enlazamos el nodo inicial y final al grafo

    grafo = Generador_Aristas(grafo, celdillas, 1);

end

% imshow(img)

%% Asignación del inicio y fin

nodo_fuente = -1;
umbral = 0;
while nodo_fuente == -1
    
    dist = inf;
    umbral = umbral + 5;
    j = size(grafo,2);
    while ((dist > umbral) && (j>=1))

        % Tomo la posición del robot
        pos = Xk;
        x_act = grafo(j).pos(1);
        inc_x = x_act - 41.13*pos(1);
        y_act = grafo(j).pos(2);
        inc_y = y_act + 41.13*pos(2);

        % Distancia al punto de destino
        dist = sqrt(inc_x^2+inc_y^2);

        if dist < umbral
            nodo_fuente = j
        end

        j = j - 1;

    end
end

% Nodo destino solicitado
destino = -1;
while (destino<0) || (destino>3)
    destino = input('\nHabitacion de destino: ');
end

switch destino
    case 0
        nodo_destino = home;
    case 1
        nodo_destino = hab_1;
    case 2
        nodo_destino = hab_2;
    case 3
        nodo_destino = hab_3;
end

while (nodo_destino == nodo_fuente) || (destino<0) || (destino>3)
    
    disp('Estás en esa posición.');
    destino = input('\nHabitacion de destino: ');
    
    switch destino
        case 0
            nodo_destino = home;
        case 1
            nodo_destino = hab_1;
        case 2
            nodo_destino = hab_2;
        case 3
            nodo_destino = hab_3;
    end
    
end
    

%% Algoritmo A*
% Inicializo los vértices
for i = 1:size(grafo,2)
    grafo(i).distancia = inf;
    grafo(i).padre = -1;
    grafo(i).visto = 0;
    
    % Heuristica (distancia euclidea)
    dist_x = grafo(i).pos(1) - grafo(nodo_destino).pos(1);
    dist_y = grafo(i).pos(2) - grafo(nodo_destino).pos(2);
    grafo(i).heuristico = sqrt(dist_x^2 + dist_y^2);
    grafo(i).estimacion = inf;
end

grafo(nodo_fuente).distancia = 0;
cola(1) = nodo_fuente;
grafo(nodo_fuente).estimacion = grafo(nodo_fuente).heuristico;

while(cola ~= 0)
    
    % Selección del mejor
    min = inf;
    for i = 1:size(cola,2)
        if (grafo(cola(i)).estimacion < min)
            min = grafo(cola(i)).estimacion;
            u = cola(i);
            quitar = i;
        end
    end
    
    % Nodo explorado
    grafo(u).visto = 1;
    
    % Lo elimino de la cola
    cola = horzcat(cola(1:(quitar-1)),cola((quitar+1):end));

    % Busco la siguiente ronda de vértices
    for i = 1:size(grafo(u).adj,2)
        
        v = grafo(u).adj(i);
        coste = grafo(u).pesos(i);
        
        % Estimación del coste para llegar a destino
        if v~=0
            dist = grafo(u).distancia + coste;
            esti = grafo(v).heuristico + dist;
            if esti < grafo(v).estimacion
                grafo(v).distancia = dist;
                grafo(v).estimacion = esti;
                grafo(v).padre = u;
                cola(end+1) = v;
            end
        end
                 
    end
       
end

%% Camino a seguir desde inicio a destino

% Compruebo el camino a seguir hasta llegar al destino
nodo = nodo_destino;
camino(1) = nodo;

% Backtraking para saber el camino
while nodo ~= nodo_fuente
    nodo = grafo(nodo).padre;
    camino(end+1) = nodo;
end

% Vértices a seguir en la trayectoria
aux = camino;
for i = 1:size(camino,2)
    camino(i) = aux(size(camino,2)+1-i);
end

% Pinto los vértices de otro color
for i = 1:size(camino,2)
    a = grafo(camino(i)).pos(2);
    b = grafo(camino(i)).pos(1);
    img(a,b,1) = 255;
    img(a,b,2) = 0;
    img(a,b,3) = 0;
end

camino

img = Muestra_Camino(grafo, camino, img);

% imshow(img)
imwrite(img,'Trayectoria.png')

Controlador(grafo, camino);

