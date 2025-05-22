function [pos,v_l,a_l,ori,v_r,a_r] = cinematica_dir(robot,q,dq,ddq,t,secuencia)
% CINEMATICA_DIR Calcula la cinemática directa y diferencial del efector final.
%
%   Calcula la posición, orientación (ángulos de Euler), velocidad lineal,
%   velocidad angular, aceleración lineal y aceleración angular del
%   efector final del robot a lo largo de una trayectoria temporal.
     
for k = 1:length(t)
%             Actualiza la configuración del robot con los valores articulares 
    robot = actualizar_robot(robot, q(:,k));
%             Extraer la posición y orientación del efector final
    posicion(:,k) = robot.T(1:3,4,end);
    R(:,:,k) = robot.T(1:3,1:3,end);
%             Obtener la orientación en ángulos de Euler a partir de la matriz R del efector final
    orientacion(:,k) = rotMat2euler(R(:, :, k),secuencia);
%             Calcular el Jacobiano geométrico a partir de la transformación global
    [Jv(:,:,k),Jw(:,:,k)] = jac_geometrico(robot);
%             Calcular la velocidad lineal y angular
            
    vel_linear(:,k)  = Jv(:,:,k) * dq(:,k);
    vel_angular(:,k) = Jw(:,:,k) * dq(:,k);
%             Calcular la derivada temporal del Jacobiano usando diferencias finitas
    if k > 1
        dJv(:,:,k) = (Jv(:,:,k) - Jv(:,:,k-1)) / dt;
        dJw(:,:,k) = (Jw(:,:,k) - Jw(:,:,k-1)) / dt;
    end
%             Calcular la aceleración lineal y angular
            
    acel_linear(:,k) = (Jv(:,:,k)*ddq(:,k))+(dJv(:,:,k)*dq(:,k));
    acel_angular(:,k) = (Jw(:,:,k)*ddq(:,k))+(dJw(:,:,k)*dq(:,k));
end
%   Entradas:
%       robot      - Estructura del robot (con NGDL, etc., y funciones aux.)
%       q          - Matriz de posiciones articulares (NGDL x M)
%       dq         - Matriz de velocidades articulares (NGDL x M)
%       ddq        - Matriz de aceleraciones articulares (NGDL x M)
%       t          - Vector de tiempo (1 x M)
%       secuencia  - String con la secuencia de ángulos de Euler (ej. 'ZYX')
%
%   Salidas:
%       pos        - Posición cartesiana (3 x M)
%       v_l        - Velocidad lineal (3 x M)
%       a_l        - Aceleración lineal (3 x M)
%       ori        - Orientación en ángulos de Euler (3 x M)
%       v_r        - Velocidad angular (3 x M)
%       a_r        - Aceleración angular (3 x M)
