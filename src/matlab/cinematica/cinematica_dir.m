function [pos,v_l,a_l,ori,v_r,a_r] = cinematica_dir(robot,q,dq,ddq,t,secuencia)
% CINEMATICA_DIR Calcula la cinemática directa y diferencial del efector final.
%
%   Calcula la posición, orientación (ángulos de Euler), velocidad lineal,
%   velocidad angular, aceleración lineal y aceleración angular del
%   efector final del robot a lo largo de una trayectoria temporal.
     
pos = zeros(3,length(t));
ori = pos;
v_l = pos;
v_r = pos;
a_l = pos;
a_r = pos;
R = zeros(3,3,length(t));
Jv = zeros(3,robot.NGDL,length(t));
dJv = Jv;
Jw = Jv;
dJw = Jw;
% Calcular velocidad y aceleración
% Algo que cabe destacar es que se realizarán cálculos discretos, por lo que en vez de una función , donde podemos sustiruir por ejemplo , aquí usaremos los índices , los  cuales se refieren a los índices del vector de tiempo que corresponden al tiempo actual , a un tiempo anterior  o al tiempo siguiente .
for k = 1:length(t)
%             Actualiza la configuración del robot con los valores articulares 
    robot = actualizar_robot(robot, q(:,k));
%             Extraer la posición y orientación del efector final
    pos(:,k) = robot.T(1:3,4,end);
    R(:,:,k) = robot.T(1:3,1:3,end);
%             Obtener la orientación en ángulos de Euler a partir de la matriz R del efector final
    ori(:,k) = rotMat2euler(R(:, :, k),secuencia);
%             Calcular el Jacobiano geométrico a partir de la transformación global
    [Jv(:,:,k),Jw(:,:,k)] = jac_geometrico(robot);
%             Calcular la velocidad lineal y angular
            
    v_l(:,k)  = Jv(:,:,k) * dq(:,k);
    v_r(:,k) = Jw(:,:,k) * dq(:,k);
%             Calcular la derivada temporal del Jacobiano usando diferencias finitas
    if k > 1
        dt = t(k)-t(k-1);
        dJv(:,:,k) = (Jv(:,:,k) - Jv(:,:,k-1)) / dt;
        dJw(:,:,k) = (Jw(:,:,k) - Jw(:,:,k-1)) / dt;
    end
%             Calcular la aceleración lineal y angular
            
    a_l(:,k) = (Jv(:,:,k)*ddq(:,k))+(dJv(:,:,k)*dq(:,k));
    a_r(:,k) = (Jw(:,:,k)*ddq(:,k))+(dJw(:,:,k)*dq(:,k));
end
