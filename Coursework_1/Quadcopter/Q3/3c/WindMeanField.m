function M_w_c = WindMeanField(u_20, p_z)
% Inputs; u_20: Wind magnitude at 20 feet
        % p_z: Altitude of wind platform
% Outputs; M_w_c Mean wind field 
    M_w_c = [u_20 * (log10(p_z/0.15)/log10(20/0.15)); 0; 0]; 
    %Vary u_20 to see what drone can handle
end