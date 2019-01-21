function [img] = Muestra_Camino(grafo, camino, img)

    for j=2:size(camino,2)
    % 'j' nodo a comprobar
                      
        % Distancia euclidea
        inc_x = grafo(camino(j)).pos(1)-grafo(camino(j-1)).pos(1);
        inc_y = grafo(camino(j)).pos(2)-grafo(camino(j-1)).pos(2);

        % Puntos iniciales para comprobar
        x_act = grafo(camino(j-1)).pos(1);
        y_act = grafo(camino(j-1)).pos(2);
        x_aux = x_act;
        y_aux = y_act;

        if abs(inc_x) >= abs(inc_y)
            
            pendiente = inc_y/inc_x;

            % Interpola al siguiente paso
            o = 0;
            if inc_x<0
                inc = -1;
            else
                inc = 1;
            end

            while o<abs(round(inc_x))

                o = o + 1;
                x_aux = x_aux + inc;
                x_act = round(x_aux);
                y_aux = y_aux + inc*pendiente;
                y_act = round(y_aux);

                % Pinto el camino
                img(y_act,x_act,1) = 255;
                img(y_act,x_act,2) = 0;
                img(y_act,x_act,3) = 0;

            end

        else
            pendiente = inc_x/inc_y;

            % Interpola al siguiente paso
            o = 0;
            if inc_y<0
                inc = -1;
            else
                inc = 1;
            end

            while o<abs(round(inc_y))

                o = o + 1;
                y_aux = y_aux + inc;
                y_act = round(y_aux);
                x_aux = x_aux + inc*pendiente;
                x_act = round(x_aux);

                % Pinto el camino
                img(y_act,x_act,1) = 255;
                img(y_act,x_act,2) = 0;
                img(y_act,x_act,3) = 0;
                
            end

        end

    end

end