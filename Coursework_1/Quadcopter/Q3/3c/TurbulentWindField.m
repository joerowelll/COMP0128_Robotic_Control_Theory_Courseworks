function T_w_c = TurbulentWindField(p_z, h, u_20, V, T, w_u, w_v, w_w)
%Inputs; p_z: Altitude of wind platform
    % h: Height of drone
    % u_20: Magnitude of wind at 20 feet
    % V: norm of velocity matrix of drone 
    % T: Time interval
    % w_u, w_v, w_w wind at current time

%Outputs: T_w_c: Wind vector at time t+T
    %Turbulent Scale lengths L
    L_u = p_z/(0.177 + 0.000823*h)^1.2;
    L_v = L_u;
    L_w = p_z;
    
    %Turbulence intensities
    sigma_w = 0.1*u_20;
    sigma_u = sigma_w/(0.177 + 0.000823*h)^0.4;
    sigma_v = sigma_u;
        
    %Epsilon White noise pertubations with variance sigma^2
    % noise = sqrt(variance) *randn(1);
    eps_u = sigma_w * randn(1);
    eps_v = sigma_u * randn(1);
    eps_w = sigma_v * randn(1);
    
    % wind strength at time t+T:
    T_w_c = [ (1 - ( V * T / L_u )) * w_u + sqrt( 2 * V * T / L_u ) * eps_u;
              (1 - ( V * T / L_v )) * w_v + sqrt( 2 * V * T / L_v ) * eps_v;
              (1 - ( V * T / L_w )) * w_w + sqrt( 2 * V * T / L_w ) * eps_w];
end