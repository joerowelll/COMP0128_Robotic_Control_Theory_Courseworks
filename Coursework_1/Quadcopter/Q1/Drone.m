%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos 
%%%%  Code modified by Jason Zhang, Ela Kanani and Joseph Rowell for
%%%%  Coursework Q1
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [3 3 0.0];

        % size of floating window that follows drone
        axis_size = 2.;

        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];

        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
    end
    properties
        % %time interval for simulation (seconds)
        time_interval

        %axis to draw on
        axis

        %length of one side of the flight arena
        spaceDim

        %limits of flight arena
        spaceLimits

        %drone position
        pos
        pos_dot

        %drone rotation
        R
        omega
        th
        thd

        %Simulation time
        time

        %parameter to start drone in random position
        pos_offset

        %number of drones
        num_drones

        %mass
        m

        %gravity
        g

        %frictional coefficient
        kd

        %propeller constant
        k
        b

        %pid constants
        Kd
        Ke
        Kp

        %length from each propeler to the centre
        L

        %Quadcopter rotational inertia matrix
        I

        %propeller inputs
        inputs
        equ_inputs

        % keeping track of position, orientation and time for plots
        xyzpos
        orientation
        times

    end
    methods

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones, time_interval)
            if nargin > 1
                obj.time_interval = time_interval;

                obj.axis = axis;

                obj.spaceDim = spaceDim;

                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 ...
                    (-spaceDim/2)+10 ...
                    (spaceDim/2)-10 ...
                    10 ...
                    spaceDim-10];

                obj.pos = [0; 0; 5];

                obj.pos_dot = zeros(3,1);

                obj.R = eye(3);

                obj.th =  zeros(3,1);

                obj.thd =  zeros(3,1);

                obj.omega = [1, 0, -sin(obj.th(2));
                    0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                    0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))
                    ] * obj.thd;

                obj.time = 0;

                obj.num_drones = num_drones;

                %constants provided

                %mass
                obj.m = 0.2;

                %gravitational acceleration
                obj.g = 9.2;

                %friction constant
                obj.kd = 0.1;

                % propeller constants
                obj.k = 1;

                obj.L = 0.2;

                obj.b = 0.1;
                
                % Rotational Inertia matrix 
                obj.I = [1 , 0, 0;
                    0, 1, 0;
                    0, 0, 0.5];
                
                % inputs
                obj.inputs = zeros(1,4);
                obj.equ_inputs = zeros(1,4);

                % keeping track of position, orientation and time
                obj.xyzpos = [];
                obj.orientation = [];
                obj.times = [];

            else
                error('Drone not initialised correctly')
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;

            %set to false if you want to see world view
            if(obj.drone_follow)
                axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL...
                    obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end

            %create middle sphere
            [X, Y, Z] = sphere(8);
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));

            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X, Y, Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end

        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function simulation_dynamics(obj)
            dt = obj.time_interval;

            %Compute new pose
            obj.omega = obj.omega + dt * angular_acceleration(obj);
            obj.thd = omega2thetadot(obj);
            obj.th = obj.th + dt * obj.thd;
            obj.pos_dot = obj.pos_dot + dt * acceleration(obj);
            obj.pos = obj.pos + dt * obj.pos_dot;
            obj.omega = thetadot2omega(obj);

            % keep track of current and previous position, orientation and
            % time
            obj.xyzpos = [obj.xyzpos,obj.pos];
            obj.orientation = [obj.orientation, obj.th];
            obj.times = [obj.times, obj.time];
        end

        %%
        function inputs = solveInputs(a_target, obj)
            %Inputs; a_target: The target z acceleration of the drone 
            %obj: The global variables specified at beginning of file

            %Outputs; Inputs: the gamma_1 to gamma_4 squared angular
            %velocity of each propeller
            gravity=[0; 0; -obj.g];
            Fd = -obj.kd * obj.pos_dot;
            obj.R = rotation(obj);
            T = Fd+ obj.m * (a_target-gravity) / (cos(obj.th(2))*cos(obj.th(1)));
            
            inputs = zeros(4, 1);
            inputs(:) = T(3)/4;
        end

        %% Acceleration
        function a = acceleration(obj)
            % Inputs; obj: Global varibales at beginning of file
                     % g: acceleration due to gravity (-9.2m/s/s)
                     % kd: Drag coefficient 
                     % pos_dot: velocity of the drone
            %Outputs; a: Acceleration of the drone
            gravity=[0; 0; -obj.g];
            obj.R = rotation(obj);
            T = obj.R * thrust(obj.inputs, obj);
            Fd = -obj.kd * obj.pos_dot;
            a = gravity + 1 / obj.m * T + Fd;
        end

        %% Thrust
        function T = thrust(inputs, obj)
            %Inputs are for Wi^2= gamma_1 to 4
            %Outputs T Thrust Vector
            T= [0; 0; obj.k * sum(inputs)];
        end

        %% Torques
        function tau = torques(inputs, obj)
            %Inputs are values for wi^2
            % obj global variables
            % Outputs tau Torque Vector 
            tau = [
                obj.L * obj.k * (inputs(1) - inputs(3))
                obj.L * obj.k * (inputs(2) - inputs(4))
                obj.b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))];
        end

        %% Rotation
        function R = rotation(obj)
            % Inputs phi(roll), theta (pitch), psi (yaw)
            % Outputs Rotation Matrix R
            phi = obj.th(1);
            theta = obj.th(2);
            psi = obj.th(3);
            
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

        end

        %%
        function omega = thetadot2omega(obj)
            % Inputs: thetadot, first derivative of roll, pitch, yaw
            % Outputs omega
            omega = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] * obj.thd;
        end
        %%
        function omegadot = angular_acceleration(obj)
            % Inputs: Drone squared angular velocity inputs 
            % I quadcopter rotational inertia matrix 
            % Outputs: omegadot Angular acceleration vector
            tau = torques(obj.inputs, obj);
            omegadot = (obj.I) \ (tau - cross(obj.omega, obj.I * obj.omega));
        end

        %%
        function thd = omega2thetadot(obj)
            %Inputs: phi, theta, psi
            %Outputs thd
            thd = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] \ obj.omega;
        end

        %%
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;

            %update inputs as required in question 1 
            if obj.time<2
                obj.inputs = obj.equ_inputs;
            elseif obj.time>=2 && obj.time<4
                obj.inputs = 1.2 .* obj.equ_inputs;
            elseif obj.time>=4 && obj.time<=8
                obj.inputs(4) = 0;
            end

            %change position and orientation of drone
            simulation_dynamics(obj);

            %draw drone on figure
            draw(obj);
        end
    end
end
