%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos
%%%%  Code modified by Jason Zhang, Ela Kanani and Joseph Rowell for
%%%%  Coursework Q2
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [2 2 0.0];

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

        %Full state feedback variables
        eigenvalues
        K
        q3_ref %reference values for drone to follow
        checkpoints %boolean vector to see if checkpoints are reached

        %length from each propeler to the centre
        L

        %quadcopter rotatinal inertia matrix
        I

        % propeller inputs
        inputs
        equ_inputs
        pre_inputs

        % LTI parameters
        Ad
        Bd

        % state
        state
        delta_x

        % keeping track of position, orientation and time for plots
        xyzpos
        orientation
        times

        % time stamp for the hovering task
        time_stamp

        % stop sim when mission completes
        mission_complete
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

                obj.pos = [0;0;0]; %inital pos

                obj.pos_dot = zeros(3,1);

                obj.R = eye(3);

                obj.th =  zeros(3,1);

                obj.thd =  zeros(3,1);
                %noise
                
                obj.omega = [1, 0, -sin(obj.th(2));
                    0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                    0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))
                    ] * obj.thd;

                %noise
                obj.pos, obj.pos_dot, obj.th, obj.omega = noise_func(obj);

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

                % Rotational Inertia Matrix
                obj.I = [1 , 0, 0;
                    0, 1, 0;
                    0, 0, 0.5];

                % inputs
                obj.equ_inputs = solveInputs(zeros(1,3), obj);
                obj.inputs = solveInputs(zeros(1,3), obj);
                obj.pre_inputs = obj.equ_inputs;
             
                % state space
                obj.state = [obj.pos; obj.pos_dot; obj.th; obj.omega];
                obj.delta_x = zeros(12,1);

                %Full state feedback variables
                obj.eigenvalues = [0.9    0.9    0.982    0.995    0.987   0.807    0.987   0.992    0.971    0.877  0.791   0.825]; %Pick arbitrary values less than one
                obj.K = zeros(4,12); %Initialise K matrix to be found when matrices Ad and Bd are calculated later
                obj.q3_ref = [5,    5,   -5,  -5,   0,  0; % define ref points; each column represents a reference
                    5,   -5,   -5,   5,   0,  0;
                    5,    10,   6,   2,   2,  0];
                % add zero padding to make sure each column is of length 12
                % to be consistent with 12x1 x vector
                obj.q3_ref = [obj.q3_ref;zeros(9,6)];
                %checkpoints (false until conditions are met)
                obj.checkpoints = [false;false;false;false;false;false];

                % Keeping track of position and orientation
                obj.xyzpos = [];
                obj.orientation = [];

                % LTI parameters
                [obj.Ad, obj.Bd] = disc_linearisation(obj);

                obj.time_stamp = obj.time;

                obj.mission_complete = false;

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
                axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
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
        function [noisy_pos, noisy_pos_dot, noisy_theta, noisy_omega] = noise_func(obj)
                     %Magnitude of 0.01, edit this to see when it fails 
                     gaussian_magnitude = 7;
                     % Variance= 1, mean = 0
                     noise = gaussian_magnitude * (sqrt(1) * randn + 0);
                     noise = noise * ones(3,1);
                     noisy_pos = obj.pos + noise;
                     noisy_pos_dot = obj.pos_dot + noise;
                     noisy_theta = obj.th + noise;
                     noisy_omega = obj.omega + noise;
                 end

        function simulation_dynamics(obj)
            dt = obj.time_interval;

            %Compute new pose
            obj.omega = obj.omega +dt * angular_acceleration(obj);
            obj.thd = omega2thetadot(obj);
            obj.th = obj.th +dt * obj.thd;
            obj.pos_dot = obj.pos_dot + dt * acceleration(obj);
            obj.pos = obj.pos + dt * obj.pos_dot;
            obj.omega = thetadot2omega(obj);

            
           
            
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

        function fsfInputs(ref_pos, obj)
            %Calculates inputs, u, using full-state feedback when given the
            %reference position and the object
            %calculate K matrix
            obj.K = place(obj.Ad,obj.Bd,obj.eigenvalues);
            %find error (i.e. difference between x and reference position)
            e = obj.state-ref_pos;
            %disp(obj.state)
            obj.inputs = obj.equ_inputs- obj.K*e;
            %disp(obj.inputs)
            obj.inputs = obj.inputs(:);
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
        function T = thrust(input, obj)
            %Inputs are for Wi^2
            %Outputs T Thrust Vector
            T= [0; 0; obj.k * sum(input)];
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
            % Outputs R rotation Matrix
            omega = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] * obj.thd;
        end

        %%
        function omegadot = angular_acceleration(obj)
            % Inputs: Drone squared angular velocity inputs
            % I quadcopter rotational inertia matrix
            % Outputs: omegadot
            tau = torques(obj.inputs, obj);
            omegadot = (obj.I) \ (tau - cross(obj.omega, obj.I * obj.omega));
        end

        %%
        function thd = omega2thetadot(obj)
            thd = [1, 0, -sin(obj.th(2));
                0, cos(obj.th(1)), cos(obj.th(2)) * sin(obj.th(1));
                0, -sin(obj.th(1)), cos(obj.th(2)) * cos(obj.th(1))] \ obj.omega;
        end

        %%
        function update(obj)
            tol = 0.1; %tolerance for controller
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            %update inputs as required
            %if checkpoint one isn't reached
            if obj.checkpoints(1) == 0 %while the target isn't reached
                ref_pos = obj.q3_ref(:,1); %target = point 5,5,5
                disp ('aiming for [5,5,5]')
                fsfInputs(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(1) = 1;
                    obj.time_stamp = obj.time;
                end
           
            elseif obj.checkpoints(1) == 1 ...
                    && obj.checkpoints(2) == 0 %when previous target is reached
                if obj.time < obj.time_stamp+10
                    disp ('Maintaining position of the first target');
                    ref_pos = obj.q3_ref(:,1); %target = point 5,5,5
                    fsfInputs(ref_pos, obj);
                    simulation_dynamics(obj);
                else    
                    ref_pos = obj.q3_ref(:,2); %target = point 5,-5,10
                    disp ('aiming for [5,-5,10]')
                    fsfInputs(ref_pos, obj);
                    simulation_dynamics(obj);
                    if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                        obj.checkpoints(2) = 1;
                    end
                end
            
            elseif obj.checkpoints(2) == 1 ...
                && obj.checkpoints(3) == 0 %when previous target is reached
                ref_pos = obj.q3_ref(:,3); %target = point -5,-5,6
                disp ('aiming for [-5,-5,6]')
                fsfInputs(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(3) = 1;
                end
            
            elseif obj.checkpoints(3) == 1 ...
                    && obj.checkpoints(4) == 0 %when previous target is reached
                ref_pos = obj.q3_ref(:,4); %target = point -5,5,2
                disp ('aiming for [-5,5,2]')
                fsfInputs(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(4) = 1;
                end

            elseif obj.checkpoints(4) == 1 ...
                    && obj.checkpoints(5) == 0 %when previous target is reached
                ref_pos = obj.q3_ref(:,5); %target = point 0,0,2
                disp ('aiming for [0,0,2]')
                fsfInputs(ref_pos, obj);
                simulation_dynamics(obj);
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(5) = 1;
                end

            elseif obj.checkpoints(5) == 1 ...
                    && obj.checkpoints(6) == 0 %when previous target is reached
                ref_pos = obj.q3_ref(:,6); %target = point 0,0,0
                disp ('landing...')
                fsfInputs(ref_pos, obj);
                simulation_dynamics(obj);
                
                if sqrt(sum((ref_pos(1:3)-obj.pos).^2)) < tol
                    obj.checkpoints(6) = 1;
                    obj.inputs = zeros(4,1);
                    disp ('Mission Complete');
                    obj.time_stamp = obj.time;
                end

            elseif obj.checkpoints(6) == 1 && obj.time > obj.time_stamp + 3
                simulation_dynamics(obj);
                obj.mission_complete = true;   
            end

            % altitude limitor
            if obj.pos(3)<0 
                obj.pos(3) = 0;
            end

            disp(obj.pos)

            % update state
            obj.state = [obj.pos;obj.pos_dot;obj.th;obj.omega];


            %draw drone to figure
            draw(obj);


            % keep track of current and previous position, orientation and
            % time
            obj.xyzpos = [obj.xyzpos,obj.pos];
            obj.orientation = [obj.orientation, obj.th];
            obj.times = [obj.times, obj.time];
        end
    end
end
