function Pos_Kalman(v,w,t)

    % Algoritmo
    global Xk
    global Pk
    global Qk_1
    global Rk
    
    Ktotal = zeros(3);      

    % Observación de los sensores
    Zk = apoloGetOdometry('Elder');
    Zk = transpose(Zk);

    % Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;

    % Avance del robot (por odometria, estimación de la siguiente posición)
    X_k = [(Xk_1(1) + v*t*cos(Xk_1(3)+(w*t/2)));
           (Xk_1(2) + v*t*sin(Xk_1(3)+(w*t/2)));
           (Xk_1(3) + w*t)];

    Ak = [1 0 (-v*sin(Xk_1(3)+w/2));
          0 1 (v*cos(Xk_1(3)+w/2));
          0 0 1                             ];

    Bk = [(cos(Xk_1(3)+w)/2) (-0.5*v*sin(Xk_1(3)+w/2));
          (sin(Xk_1(3)+w/2)) (0.5*v*cos(Xk_1(3)+w/2));
           0                     1                   ];
       
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    Hk = Ak;
    Zk_ = Hk * X_k;

    % Comparacion
    Yk = Zk-X_k;

    for r=1:3
        if Yk(r)>pi
            Yk(r) = Yk(r) - 2*pi;
        end
        if Yk(r)<(-pi)
            Yk(r) = Yk(r) + 2*pi;
        end
    end
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')*inv(Sk);

    % Correccion
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;

end

