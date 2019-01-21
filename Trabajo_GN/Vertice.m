classdef Vertice
    % Clase vértice para definir el grafo
    properties 
        id      % Identificador del vértice
        adj     % Vértices adyacentes
        pesos   % Pesos de las aristas
        Name    % Habitación asociada
        distancia   % Profundidad del vértice respecto al inicial
        padre   % Padre en el algoritmo de búsqueda
        visto   % Explorado en el algoritmo de búsqueda
        pos     % Posición del vértice
        heuristico  % Estimación hasta llegar a destino
        estimacion  % Estimación del coste total hasta destino
    end
    methods
        % Función de inicialización
        function obj = Vertice(id_n, adj_n, pesos_n, pos_n)
            obj.id = id_n;
            obj.adj = adj_n;
            obj.pesos = pesos_n;
            obj.pos = pos_n;
        end
        function Muestra_Datos(obj)
            obj
        end
    end
end

