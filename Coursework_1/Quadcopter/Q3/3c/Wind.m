function w_total = Wind(pos_dot, p_z, prev_w, T, u_20, obj)
    % Inputs; pos_dot: Drone Velocities
            % p_z: Altitude of wind platform
            % prev_w: Previous wind vector
            % T: Time interval
            % u_20: Magnitude of wind at 20 feet
    %Outputs; w_total: total wind vector including mean wind field and
    %turbulent wind field.
    % Alter u_20 input to this to determine what drone can take 
    %Avoid log singularity by having no wind at v low altitude
    if obj.pos(3) <1
        w_total = zeros(3,1);
    else
        V = norm(pos_dot);
        M_w_c = WindMeanField(u_20, p_z);
        T_w_c = TurbulentWindField(p_z, p_z, u_20, V, T, prev_w(1),  prev_w(2), prev_w(3));
        w_total = M_w_c + T_w_c;
end