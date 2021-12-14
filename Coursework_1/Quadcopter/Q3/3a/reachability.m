%% Temporarily in a separate file
%clear clc close all
%Run after quadcopter_script to have the drone variable
% Find A and B, C and D 
[Ad, Bd] = disc_linearisation(drones);
C = eye(size(Ad));
D = zeros(size(Bd));

%% Check if full-state feedback provides a reachable solution
%Step 1: Calculate reachability matrix
Wr = reachability_matrix(Ad,Bd);
%Step 2: Calculate rank of reachability matrix 
Wr_rank = rank(Wr,1e-16);
%Step 3: compare rank to matrix dimensions to see if full rank
if Wr_rank == min(size(Wr))
    reachability = true; %if full rank, the solution is reachable
    disp('Full rank reachability matrix -- the solution is reachable.')
else 
    disp('Reachability matrix is not full rank -- the solution is unreachable.')
end


%% Implement controller

%Pick arbitrary values less than one, but we shouldn't make them all near
%one else the response will be unstable
eigenvalues = linspace(0.6,1,12);

%Find poles using A,B and the eigenvalues
K = place(Ad,Bd,eigenvalues);
% define ref points; each column represents a reference
q3_ref = [5,    5,   -5,  -5,   0,  0;
          5,   -5,   -5,   5,   0,  0;
          5,    10,   6,   2,   2,  0];
% add zero padding to make sure each column is of length 12
q3_ref = [q3_ref;zeros(9,6)];

%% First checkpoint
%raise up to (5,5,5) and stay for 10 seconds
first_checkpointpoint_reached = false;
%initalise x

while first_checkpointpoint_reached == false
    x = [drones.pos; drones.pos_dot; drones.th; drones.omega];
    e = x-q3_ref(:,1); %want to reach (5,5,5)
    drones.state = -K*e;
    disp('pos')
    disp(drones.pos)
    disp('u')
    disp(drones.state)
     if e == 0 %&& dt == 10
         first_checkpointpoint_reached = true; %when (5,5,5 is reached) for 10 seconds
     end
end




%% Functions

%Calculate the reachability matrix (Wr) using A and B from the discrete
%linearisation (LTI) form, where Wr = [B AB ... A^(T-1)B]

function Wr = reachability_matrix(A,B)
    %Inputs A and B jacobian matrices 
    % Outputs Wr Reachability matrix
    Wr =[]; %initialise matrix
    Wr(:,1:4) = B; %add B matrix first
    for T = 1:size(A,1)-1
        AB = A^(T)*B; 
        Wr = [Wr, AB]; 
    end
end