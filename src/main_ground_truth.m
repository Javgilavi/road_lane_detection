close all; clear all;

ground_truth = [];
% Lectura del vídeo:
mivideo = VideoReader('..\videos_input\autopista buena visibilidad.mp4');

% Cada cuantos frames actualiza.
act_frame = 5;

%Inicializo matrices de covarianza y estado
Q_izq_est = [];
P_izq_est = [];
Q_dcha_est = [];
P_dcha_est = [];

%Constantes para filtro Kalman
dt = 0.75;
u = [0.01; 0.01];
acc_noise = 0.1;
c_meas_noise = 0.1;
theta_meas_noise = 0.1;
Ez = [c_meas_noise, 0; 0, theta_meas_noise]; % measurement prediction error
Ex = [(dt^4)/4, 0, (dt^3)/2, 0; 0, (dt^4)/4, 0, (dt^3)/2; (dt^3)/2, 0, dt^2, 0; 0, (dt^3)/2, 0, dt^2] * (acc_noise^2); % State prediction error    
% state and measurement equations
A = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1];
B = [(dt^2)/2, 0; 0, (dt^2)/2; dt, 0; 0, dt];
C = [1, 0, 0, 0; 0, 1, 0, 0];

%Contadores uso errores kalman
cnt_izq = 0;
cnt_dcha = 0;

% Listas para grabar variables de las lineas rectas
rho_izq_xData=[];
rho_izq_yData=[];
rho_izq_xData2=[];
rho_izq_yData2=[];

theta_izq_xData=[];
theta_izq_yData=[];
theta_izq_xData2=[];
theta_izq_yData2=[];

rho_dcha_xData=[];
rho_dcha_yData=[];
rho_dcha_xData2=[];
rho_dcha_yData2=[];

theta_dcha_xData=[];
theta_dcha_yData=[];
theta_dcha_xData2=[];
theta_dcha_yData2=[];

desviacion_xData=[];
desviacion_yData=[];

%Inicializo variable de error
er = 3;

%Leemos frame actual
for i = 1:mivideo.NumFrames

    f_video = read(mivideo, i);    

    % Solo actualizo las lineas cada [act_frames] frames
    if(mod(i-1,act_frame) == 0)
        f = read(mivideo, i);
        %Reduzco el tamaño de la imagen para acelerar el procesado
        %f = imresize(f,0.2);
        %f = imnoise(f,"gaussian",0,0.01);

        [heigh_im, wide_im] = size(f(:,:,1));

        %Filtro de Kalman
        if exist('Q_izq', 'var') && exist('P_izq', 'var')
            [Q_izq_est, P_izq_est] = Kalman_filter_Estimate(A, B, u, Ex, Q_izq, P_izq);
        end
    
        if exist('Q_dcha', 'var') && exist('P_dcha', 'var')
            [Q_dcha_est, P_dcha_est] = Kalman_filter_Estimate(A, B, u, Ex, Q_dcha, P_dcha);
        end
        
        % Detecto las lineas
        if er == 3      %Caso inicial, cuando aun no hya estimación
            [line_izq, line_dcha] = detect_lane(f, 0, 0);
            rho_izq_xData2 = [rho_izq_xData2 i];
            rho_izq_yData2 = [rho_izq_yData2 line_izq.rho];
            theta_izq_xData2 = [theta_izq_xData2 i];
            theta_izq_yData2 = [theta_izq_yData2 line_izq.theta];
            rho_dcha_xData2 = [rho_dcha_xData2 i];
            rho_dcha_yData2 = [rho_dcha_yData2 line_dcha.rho];
            theta_dcha_xData2 = [theta_dcha_xData2 i];
            theta_dcha_yData2 = [theta_dcha_yData2 line_dcha.theta]; 
            er = 0;
        elseif ~isempty(Q_izq_est) && ~isempty(Q_dcha_est) && er==0   %Si se ha calculado ya estimacion y no hubo error
            [line_izq, line_dcha] = detect_lane(f, Q_izq_est, Q_dcha_est);

            if ~isempty(line_izq)
                if(abs(line_izq.rho - Q_izq_est(1)) > 30 || abs(line_izq.theta - Q_izq_est(2)) > 15)
                    line_izq = [];
                    er = 1;
                else
                    rho = Q_izq_est(1);
                    theta = Q_izq_est(2);
                    rho_izq_xData2 = [rho_izq_xData2 i];
                    rho_izq_yData2 = [rho_izq_yData2 rho];
                    theta_izq_xData2 = [theta_izq_xData2 i];
                    theta_izq_yData2 = [theta_izq_yData2 theta];
                    cnt_izq = 0;
                end
            end

            if(~isempty(line_dcha))
                if(abs(line_dcha.rho - Q_dcha_est(1)) > 30 && abs(line_dcha.theta - Q_dcha_est(2)) > 15)
                    line_dcha = [];
                    er = 1;
                else
                    rho = Q_dcha_est(1);
                    theta = Q_dcha_est(2);
                    rho_dcha_xData2 = [rho_dcha_xData2 i];
                    rho_dcha_yData2 = [rho_dcha_yData2 rho];
                    theta_dcha_xData2 = [theta_dcha_xData2 i];
                    theta_dcha_yData2 = [theta_dcha_yData2 theta];
                    cnt_dcha = 0;
                end
            end

        else    %Ha sucedido un error
            [line_izq, line_dcha] = detect_lane(f, 0, 0);
            
            if ~isempty(line_izq)
                if(abs(line_izq.rho - Q_izq_est(1)) > 30 || abs(line_izq.theta - Q_izq_est(2)) > 15)
                    line_izq = [];
                    er = 1;
                else
                    rho_izq_xData2 = [rho_izq_xData2 i];
                    rho_izq_yData2 = [rho_izq_yData2 line_izq.rho];
                    theta_izq_xData2 = [theta_izq_xData2 i];
                    theta_izq_yData2 = [theta_izq_yData2 line_izq.theta];
                    er = 0;
                    cnt_izq = 0;
                end
            else
                er = 1;
            end
            if(~isempty(line_dcha))
                if(abs(line_dcha.rho - Q_dcha_est(1)) > 30 && abs(line_dcha.theta - Q_dcha_est(2)) > 15)
                    line_dcha = [];
                    er = 1;
                else
                    rho_dcha_xData2 = [rho_dcha_xData2 i];
                    rho_dcha_yData2 = [rho_dcha_yData2 line_dcha.rho];
                    theta_dcha_xData2 = [theta_dcha_xData2 i];
                    theta_dcha_yData2 = [theta_dcha_yData2 line_dcha.theta]; 
                    er = 0;
                    cnt_dcha = 0;
                end
            else 
                er = 1;
            end
        end

        if isempty(line_izq)
            rho = Q_izq_est(1);
            theta = Q_izq_est(2);
            x1 = (rho - heigh_im * sind(theta)) / cosd(theta); % Coordenada x del punto de inicio de la línea
            y1 = heigh_im;                                   % Coordenada y del punto de inicio de la línea
            x2 = (rho) / cosd(theta);                            % Coordenada x del punto final de la línea
            y2 = 1;                                             % Coordenada y del punto final de la línea
            line_izq = struct('point1', [x1, y1], 'point2', [x2, y2], 'theta', theta, 'rho', rho);
            rho_izq_xData2 = [rho_izq_xData2 i];
            rho_izq_yData2 = [rho_izq_yData2 line_izq.rho];
            theta_izq_xData2 = [theta_izq_xData2 i];
            theta_izq_yData2 = [theta_izq_yData2 line_izq.theta];
            cnt_izq=cnt_izq+1;
            er = 1;
            if cnt_izq >= 10    %Reinicio kalman
                Q_izq_est = [];
                P_izq_est = [];
                Q_dcha_est = [];
                P_dcha_est = [];
                er = 3;
            end
        end

        if isempty(line_dcha)
            rho = Q_dcha_est(1);
            theta = Q_dcha_est(2);
            x1 = (rho - heigh_im * sind(theta)) / cosd(theta);   % Coordenada x del punto de inicio de la línea
            y1 = heigh_im;                                       % Coordenada y del punto de inicio de la línea
            x2 = (rho) / cosd(theta);                               % Coordenada x del punto final de la línea
            y2 = 1;                                                 % Coordenada y del punto final de la línea
            line_dcha = struct('point1', [x1, y1], 'point2', [x2, y2], 'theta', theta, 'rho', rho);
            rho_dcha_xData2 = [rho_dcha_xData2 i];
            rho_dcha_yData2 = [rho_dcha_yData2 line_dcha.rho];
            theta_dcha_xData2 = [theta_dcha_xData2 i];
            theta_dcha_yData2 = [theta_dcha_yData2 line_dcha.theta];
            cnt_dcha=cnt_dcha+1;
            er = 1;
            if cnt_dcha >= 10    %Reinicio kalman
                Q_izq_est = [];
                P_izq_est = [];
                Q_dcha_est = [];
                P_dcha_est = [];
                er = 3;
            end
        end

        rho_izq_xData = [rho_izq_xData i];
        rho_izq_yData = [rho_izq_yData line_izq.rho];
        theta_izq_xData = [theta_izq_xData i];
        theta_izq_yData = [theta_izq_yData line_izq.theta];
    
        rho_dcha_xData = [rho_dcha_xData i];
        rho_dcha_yData = [rho_dcha_yData line_dcha.rho];
        theta_dcha_xData = [theta_dcha_xData i];
        theta_dcha_yData = [theta_dcha_yData line_dcha.theta]; 

        %Actualizo filtro de Kalman
        if er~=3
            [Q_izq, P_izq] = Kalman_Filter_Update(C, Ex, Ez, Q_izq_est, P_izq_est, [line_izq.rho; line_izq.theta]);
            [Q_dcha, P_dcha] = Kalman_Filter_Update(C, Ex, Ez, Q_dcha_est, P_dcha_est, [line_dcha.rho; line_dcha.theta]); 
        end

        %Reescalo las lineas
%         line_izq.rho = line_izq.rho*5; 
%         line_dcha.rho = line_dcha.rho*5;
        desviacion_cm = draw_lanes(f_video,[line_izq, line_dcha], ['green', 'red']);
        

        % Saco valores Ground Truth
        rho_izq_gt = line_izq.rho;
        theta_izq_gt = line_izq.theta;
        rho_dcha_gt = line_dcha.rho;
        theta_dcha_gt = line_dcha.theta;

        % Ask user whether lines were detected correctly
        user_input = input('¿Deteccion correcta? (y/n): ', 's');
        
        % Handle user input
        if strcmpi(user_input, 'n')
             user_input = input('¿Que linea fallo? (id/i/d): ', 's');
             if strcmpi(user_input, 'id')
                 [x_correct,y_correct] = ginput(2);
                 x1=x_correct(1); y1=y_correct(1);
                 x2=x_correct(2); y2=y_correct(2);
                 dx = x2 - x1;
                 dy = y2 - y1;
                 rho_izq_gt = abs(x1 * dy - y1 * dx) / sqrt(dx^2 + dy^2);
                 theta_izq_gt = atan2(dy, dx);

                 [x_correct,y_correct] = ginput(2);
                 x1=x_correct(1); y1=y_correct(1);
                 x2=x_correct(2); y2=y_correct(2);
                 dx = x2 - x1;
                 dy = y2 - y1;
                 rho_dcha_gt = abs(x1 * dy - y1 * dx) / sqrt(dx^2 + dy^2);
                 theta_dcha_gt = atan2(dy, dx);  

             elseif strcmpi(user_input, 'i')
                 [x_correct,y_correct] = ginput(2);
                 x1=x_correct(1); y1=y_correct(1);
                 x2=x_correct(2); y2=y_correct(2);
                 dx = x2 - x1;
                 dy = y2 - y1;
                 rho_izq_gt = abs(x1 * dy - y1 * dx) / sqrt(dx^2 + dy^2);
                 theta_izq_gt = atan2(dy, dx);
             else
                 [x_correct,y_correct] = ginput(2);
                 x1=x_correct(1); y1=y_correct(1);
                 x2=x_correct(2); y2=y_correct(2);
                 dx = x2 - x1;
                 dy = y2 - y1;
                 rho_dcha_gt = abs(x1 * dy - y1 * dx) / sqrt(dx^2 + dy^2);
                 theta_dcha_gt = atan2(dy, dx);  

             end
            
        end
        nueva_linea = [rho_izq_gt,theta_izq_gt,rho_dcha_gt,theta_dcha_gt];
        ground_truth = [ground_truth;nueva_linea];

    end

end

save('Ground Truths\video1.mat','ground_truth');
