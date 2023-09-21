%function para determinar el estado y covarianza del sistema usando el algoritmo de Kalman
function  [Q,P] =  Kalman_Filter_Update(C,Ex,Ez,Q_est,P_est,z)

    if size(Q_est)   

        %Step 3: Kalman_gain_at_t = Est_of_state_cov_at_t * C_transpose ( C * Est_of_state_cov_at_t * C_transpose   +  Ez  )
        % Ganancia de Kalman (Peso que se le debe dar para la discrepancia en la medida y predicción para actualizar el estado y covarianza)
	    K = P_est*C'*inv(C*P_est*C'+Ez);
    
	    %Step 4: State_at_t = Est_of_state_at_t + Kalman_gain_at_t (measured_variable_at_t - (C * Est_of_state_at_t) )
        % Corrección del estado estimado usando las mediciones para obtener estado actual
        Q = Q_est + K*(z - C*Q_est);
    
	    %Step 5: State_cov_at_t = (I - Kalman_gain_at_t * C) Est_of_state_cov_at_t
        % Correción covarianza estimada usando las mediciones para obtener covarianza actual
        P = (eye(4) - K*C)* P_est;  
    else
        % Variable de estado [[rho],[theta],[rho'],[theta']]
        Q = [z(1,1); z(2,1); 0; 0];
        % Estimación de la matriz de covarianza inicial
		P = Ex;
    end
end
