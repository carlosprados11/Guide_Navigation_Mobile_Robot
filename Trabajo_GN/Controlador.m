function Controlador(grafo, camino)

    load('Q.mat');
    load('R.mat');
    cuenta = 1;
    
    % Umbrales
    umbral = 2;
    umbral_ang = 0.001;

    Qd = Q(1,1);
    Qb = Q(2,2);
    global Qk_1
    Qk_1 = Q;
    
    % Posición
    global Xk
    global Pk
    
    % Varianza en la medida (de los ultrasonidos)
    global Rk
    Rk = R;
    
    % Trayectoria a seguir
    for i = 2:1:size(camino,2)
        
        dist = inf;
        
        % Tomo la posición del robot
        pos = Xk;
        x_act = grafo(camino(i)).pos(1);
        inc_x = x_act - 41.135*(0.5+pos(1));
        y_act = grafo(camino(i)).pos(2);
        inc_y = y_act + 41.135*(pos(2)-0.5);
                
        % Angulo deseado
        if inc_x>0
            alpha_f = atan(-inc_y/inc_x);
        else
            alpha_f = atan(inc_y/inc_x);
            if inc_y>0
                alpha_f = -pi - alpha_f;
            else
                alpha_f = pi - alpha_f;
            end
        end
        
        % Angulo actual
        alpha_i = pos(3);
        
        % Giro del robot
        v = 0;
        bien = 0;
        cnt = 1;
        while (abs(alpha_i-alpha_f) > umbral_ang) %&& (((alpha_i<pi-umbral_ang)&&(alpha_f>-pi+umbral_ang))||((alpha_f<pi-umbral_ang)&&(alpha_i>-pi+umbral_ang))))
            if (alpha_f-alpha_i)>0
                w = cnt*1;
            else
                w = -cnt*1;
            end
            if (((alpha_i>pi/2)&&(alpha_f<-pi/2))||((alpha_i<-pi/2)&&(alpha_f>pi/2)))
                if alpha_i<-pi/2
                    w = -cnt*1;
                else
                    w = cnt*1;
                end
            end
            % Giro del robot
            apoloMoveMRobot('Elder',[v,w],0.001);
            apoloUpdate();
            pause(0.001);
            
            % Posición estimada
            Pos_Kalman(v,w,0.001);
            pos = Xk;
            alpha_i = pos(3);
            
            bien = bien + 1;
            if bien>10000
                cnt = -cnt;
                bien = 0;
            end
        end
                
        % Mientras no hayamos llegado al siguiente punto
        salir = 0;
        while ((abs(dist) > umbral)&&(salir<10))

            % Incrementos de distancia a realizar
            x_act = grafo(camino(i)).pos(1);
            inc_x = x_act - 41.13*(0.5+pos(1));
            y_act = grafo(camino(i)).pos(2);
            inc_y = y_act + 41.13*(-0.5+pos(2));
            
            % Distancia al punto de destino
            dist = sqrt(inc_x^2+inc_y^2);
            
            % Angulo deseado
            if inc_x>0
                alpha_f = atan(-inc_y/inc_x);
            elseif inc_x<0
                alpha_f = atan(inc_y/inc_x);
                if inc_y>0
                    alpha_f = -pi - alpha_f;
                else
                    alpha_f = pi - alpha_f;
                end
            else
                alpha_f = -pi/2;
            end

            % Angulo actual
            alpha_i = pos(3);
            
            if abs(alpha_f-alpha_i)>2*umbral_ang
                if (alpha_f-alpha_i)>0
                    w = 1;
                else
                    w = -1;
                end
            else
                w = 0;
            end
            if (((alpha_i>pi/2)&&(alpha_f<-pi/2))||((alpha_i<-pi/2)&&(alpha_f>pi/2)))
                if alpha_i<-pi/2
                    w = -1;
                else
                    w = 1;
                end
            end
            
            
            % Giro del robot
            v = 2;
            apoloMoveMRobot('Elder',[v,w],0.001);
            apoloUpdate();
            pause(0.001);

            % Posición estimada
            Pos_Kalman(v,w,0.001);
            pos = Xk;
            Xk_acum(cuenta,:) = [pos(1)+0.5, pos(2)-0.5, pos(3)];
            Pk_acum(cuenta,:,:) = Pk;
            pos_real(cuenta,:) = apoloGetLocation('Elder');            
            cuenta = cuenta + 1;
            
        end

    end

    %% Representacion grafica 
    % Posiciones por las que pasa el robot y por las que debería pasar
    figure(1);
    subplot(1,2,1)
    hold on
    plot(pos_real(:,1),pos_real(:,2),'.k')
    plot(Xk_acum(:,1),Xk_acum(:,2),'.r');
    legend('Movimiento Real','Estimación')
    hold off

    % Orientaciones adoptadas
    subplot(1,2,2);
    hold on
    plot(pos_real(:,6),'g');
    xlabel ('t (muestras)')
    ylabel ('\theta (rad)')
    hold off

    % Varianzas en 'x', 'y', y 'ori'
    figure(2);
    subplot(3,1,1);
    plot(Pk_acum(:,1,1),'b');
    xlabel ('t (muestras)')
    ylabel ('Varianza X (m2)')
    hold on

    subplot(3,1,2);
    plot(Pk_acum(:,2,2),'b');
    xlabel ('t (muestras)')
    ylabel ('Varianza Y (m2)')

    subplot(3,1,3);
    plot(Pk_acum(:,3,3),'b');
    xlabel ('t (muestras)')
    ylabel ('Varianza \theta (rad2)')


end

