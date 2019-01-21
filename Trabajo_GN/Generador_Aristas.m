function [grafo] = Generador_Aristas(grafo, celdillas, enlazar)

    if enlazar==0
        principio = 1;
        final = size(grafo,2);
    else
        principio = size(grafo,2) - 3;
        final = size(grafo,2) - 4;
    end
    
    for j=1:final 
    % 'j' nodo a comprobar

        for m=principio:size(grafo,2)
            % 'm' nodos restantes

            if j<m
                      
                % Distancia euclidea
                inc_x = grafo(m).pos(1)-grafo(j).pos(1);
                inc_y = grafo(m).pos(2)-grafo(j).pos(2);
                dist = sqrt(inc_y^2+inc_x^2);

                if ((dist < grafo(j).pesos(size(grafo(j).pesos,2)))||(dist < grafo(m).pesos(size(grafo(m).pesos,2))))
                    
                    % Puntos iniciales para comprobar
                    x_act = grafo(j).pos(1);
                    y_act = grafo(j).pos(2);
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

                        choque = 0;
                        while ((o<abs(round(inc_x))) && (choque~=1))

                            o = o + 1;
                            x_aux = x_aux + inc;
                            x_act = round(x_aux);
                            y_aux = y_aux + inc*pendiente;
                            y_act = round(y_aux);

                            % Comprobación de choque
                            if (celdillas(y_act,x_act)==0)
                                choque = 1;
                            end

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

                        choque = 0;
                        while ((o<abs(round(inc_y))) && (choque~=1))

                            o = o + 1;
                            y_aux = y_aux + inc;
                            y_act = round(y_aux);
                            x_aux = x_aux + inc*pendiente;
                            x_act = round(x_aux);

                            % Comprobación de choque
                            if (celdillas(y_act,x_act)==0)
                                choque = 1;
                            end
                        end
                  
                    end
     
                end

                % Asigno el valor al vector de distancias más pequeñas
                if choque==0
                    
                    if dist < grafo(j).pesos(size(grafo(j).pesos,2))
                        
                        % Asigno a un nodo
                        grafo(j).pesos(size(grafo(j).pesos,2)) = dist;
                        grafo(j).adj(size(grafo(j).adj,2)) = m;

                        % Ordeno los valores minimos de uno
                        aux_min = grafo(j).pesos;
                        aux_nod = grafo(j).adj;
                        grafo(j).pesos = sort(grafo(j).pesos);
                        for n=1:size(grafo(j).adj,2)
                            for o=1:size(grafo(j).adj,2)
                                if grafo(j).pesos(n)==aux_min(o)
                                    grafo(j).adj(n) = aux_nod(o);
                                end
                            end
                        end
                    end

                    % Compruebo si es de las distancias menores en el otro nodo
                    if dist < grafo(m).pesos(size(grafo(m).pesos,2))

                        % Asigno al otro
                        grafo(m).pesos(size(grafo(m).pesos,2)) = dist;
                        grafo(m).adj(size(grafo(m).adj,2)) = j;

                        % Y del otro
                        aux_min = grafo(m).pesos;
                        aux_nod = grafo(m).adj;
                        grafo(m).pesos = sort(grafo(m).pesos);
                        for n=1:size(grafo(m).pesos,2)
                            for o=1:size(grafo(m).pesos,2)
                                if grafo(m).pesos(n)==aux_min(o)
                                    grafo(m).adj(n) = aux_nod(o);
                                end
                            end
                        end
                        
                    end
                end
            end
        end  

    end

end

