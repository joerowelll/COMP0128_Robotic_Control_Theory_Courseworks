function [Ad, Bd] = disc_linearisation(obj)
    % symbolic variables used to compute analytical functions
    %{
    x 1-3: pos_x pos_y pos_z
    x 4-6: posd_x posd_y posd_z
    x 7-9: phi theta psi
    x 10-12: phid thetad psid
    
    xd 1-3: posd_x posd_y posd_z
    xd 4-6:
    xd 7-9: om1 om2 om3 % omega
    xd 10-12:
    %}
    
    syms x [12,1]
    syms xd [12,1]
    syms u [4,1]

    %% define parameters
    gravity=[0; 0; -obj.g];
    I = obj.I;
    
    %% compute dynamics
    
  
    phi = x(7);
    theta = x(8);
    psi = x(9);
    
    Rx = [1 0 0;
    0 cos(phi) -sin(phi); ... 
    0 sin(phi) cos(phi)];
    
    Ry = [ cos(theta) 0 sin(theta);
    0 1 0; ...
    -sin(theta) 0 cos(theta)];
    
    Rz = [cos(psi) -sin(psi) 0; ... 
        sin(psi) cos(psi) 0; ... 
        0 0 1];

    R = Rz*Ry*Rx;

    Fd = -obj.kd * [x(4); x(5); x(6)] ;
    
    tau = [
    obj.L * obj.k * (u(1) - u(3))
    obj.L * obj.k * (u(2) - u(4))
    obj.b * (u(1) - u(2) + u(3) - u(4))];

    Tb = [0; 0; obj.k * sum(u)];
    
    %% state space
    xd(1:3) = x(4:6);
    xd(4:6) = gravity +(1/obj.m) * R *(Tb) + (1/obj.m) *Fd;
    xd(7:9) = [1, 0, -sin(x(8));
        0, cos(x(7)), cos(x(8)) * sin(x(7));
        0, -sin(x(7)), cos(x(8)) * cos(x(7))] \ x(10:12);
    xd(10:12) = I\tau - [...
        (I(2,2)-I(3,3))/I(1,1)*xd(8)*xd(9);...
        (I(3,3)-I(1,1))/I(2,2)*xd(7)*xd(9);...
        (I(1,1)-I(2,2))/I(3,3)*xd(7)*xd(8)];
    
    Aj = jacobian(xd,x);
    Bj = jacobian(xd,u);
    
    A =  subs(Aj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        obj.state.');
    A = double (subs(A, u, obj.inputs));
    B = double (subs(Bj,...
        [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12],...
        obj.state.'));

    C = eye(size(A));
    D = zeros(size(B));
    
    cont_sys = ss(A,B,C,D);
    disc_sys = c2d(cont_sys,obj.time_interval,'zoh');

    Ad = disc_sys.A;
    Bd = disc_sys.B;
    
end
