%function para determinar el estado y covarianza del sistema usando el algoritmo de Kalman
function  [Q_est,P_est] =  Kalman_filter_Estimate(A,B,u,Ex,Q,P)
    
	%Step 1: Est_of_x_at_t = (A * x_at_t-1) + (B * u_at_t-1)
	Q_est = A*Q + B*u; %Predicción de estado

	%Step 2: Est_of_state_cov_at_t = (A * state_cov_at_t-1 * A_transpose) + Ex
	P_est = A*P*A'+ Ex;  %Predicción covarianza
end