

function [t, q, q_act] = main1
    % main.
    
    % cleans workspace
    clear
    close all
    clc
    
    addpath('matlab/');
    addpath('functions_matlab/');
    porta = 19997; % default CoppeliaSim port
    
    % initialise robot variables
    init
    
    addpath("functions_matlab\")
    % Define the total duration and time step

    tf = 2; % in seconds
    Ts = 1e-3; % in seconds
    
    % space in which the robot performs its tasks
    % related to the Cartesian coordinate frame
    n = 7;
    
    % time vector from 0 to tf
    t = 0 : Ts: tf;
    % number of time steps
    N = length(t);

    % joint variables in DH
    % Initial joint configuration (in degrees)
    q = zeros(n, N); 
    % convert degrees to radians
    q(:, 1) = [56 51 -18 146 14 96 66]'/180*pi;
    dq = zeros(n, N);  % Joint velocities

 % Joint variables in Jaco2 7DOFs convention
    q_jaco = zeros(7,N);
    q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));

    % Quaternion orientation variables
    quat = zeros(4,N);
    quat_d = zeros(4,N);

    % error positions
    error_pos = zeros(3, N);  % Position error
    error_quat = zeros(3, N);  % Orientation error
    error = zeros(6, N);  % Combined error

    condJ = zeros(1, N);  % Condition number of the Jacobian

    %curvilinear abscissa
    s_t = zeros(N,1);
    ds_t = zeros(N,1);
    dds_t = zeros(N,1);
    s0 = 0;
    s1 = 1;
    dsc = 0.6;

    % position variables
    x   = zeros(3,N); % current position
    posd = zeros(3,N); % desired position
    dposd = zeros(3,N); % desired velocity

    % DH parameters definition
    a     = zeros(7,1);
    alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
    d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
    theta = [pi/4 pi/3 -pi/8 pi/3 -pi/12 pi/12 0]'; % 
    DH    = [a alpha d theta];


    % initial and final position definition
    DH(:,4) = q(:,1);
    T = DirectKinematics(DH);
    posf = [0.3, -0.55, 0.3]'; % final position
    pos0 = T(1:3, 4, n); % initial position
    

    
    % desired quaternion (quat_d)
     quat_d = zeros(4, N);
    for i = 1:N
     quat_d(:, i) = [1, 0, 0, 0]'; % constant desired orientation ex ey ex eta
    end
    
    
    % Select the algorithm
    algorithm = 'inverse';
    
    if strcmp(algorithm, 'inverse')
        K = 5 * diag([10 10 10 10 10 10]);
        fprintf('\n algorithm: inverse of the jacobian \n');
    else
        K = 1 * diag([9 9 12 12 12 12]);
        fprintf('\n algorithm: transpose of the jacobian \n');
    end
    

    %  Starting comunication with CoppeliaSim
        clc
        fprintf('----------------------');
        fprintf('\n simulation started ');
        fprintf('\n trying to connect...\n');
        [clientID, vrep ] = StartVrep(porta,Ts);    
        handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
        my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,1)); % first move to q0
    
    % inverse kinematics loop
    for i = 1:N
        tic
        % application of trapezoidal velocity profile to segment path
        [s_t(i), ds_t(i), dds_t(i)] = trapezoidal(s0, s1, dsc, 2, t(i));
        posd(:, i) = pos0 + (posf - pos0)*s_t(i);
        dposd(:, i) = (posf - pos0)*ds_t(i);
      

        % desired quaterion orientation
        quat_d(:, i) = [1 0 0 0]';
    
        % direct kinematics
        DH(:, 4) = q(:, i);
        T = DirectKinematics(DH);
        x(:, i) = T(1:3, 4, end);
        quat(:, i) = Rot2Quat(T(1:3, 1:3, end));
    
        % Jacobian
        J = Jacobian(DH);

        % condition of the Jacobian is calculated and stored
        condJ(i) = cond(J);
    
        % Inverse kinematics algorithm
        error_pos(:,i) = posd(:,i) - x(:,i);                % Position error
        error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i)); % Quaternion error 
        error(:,i) = [error_pos(:,i);error_quat(:,i)];      % Total error
    

        % Inverse kinematics algorithm
        if strcmp(algorithm, 'transpose')
            dq(:, i) = J' * K * error(:, i);
        else
            dq(:, i) = pinv(J) * K * error(:, i);
        end


        % euler integration
        if i<N
            q(:,i+1) = q(:,i) + Ts*dq(:,i);
        end
    
        % Joint velocities for the Jaco Model in CoppeliaSim
        q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
        my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i)); % first move to q0
    
         % Elasped Time from the iteration
        elapsedTime = toc; 
        pause(Ts-elapsedTime)
    end
    
    DeleteVrep(clientID, vrep); 

    % Plot the results
    figure
    subplot(231)
    plot(t, q'), grid on
    ylabel('q [rad]')
    title('Joint position')
    subplot(234)
    plot(t, dq'), grid on
    ylabel('dq [rad/s]')
    title('Velocity profile')
    subplot(232)
    plot(t, error(1:2, :)), grid on
    ylabel('p err [m]')
    title('Position Error')
    subplot(235)
    plot(t, error(3:5, :)), grid on
    ylabel('o err [-]')
    xlabel('time [s]')
    title('Orientation error')
    subplot(233)
    plot(t, condJ), grid on
    ylabel('cond J')
    xlabel('time [s]')
    title('Condition number of Jacobian')
    
    figure
    hold on
    DH(:, 4) = q(:, 1);
    DrawRobot(DH);
    DH(:, 4) = q(:, N);
    DrawRobot(DH);
    plot3(x(1, :), x(2, :), x(3, :), 'LineWidth',2.0)
    
    % Plot initial and final positions
    plot3(x(1, 1), x(2, 1), x(3, 1), 'ko', 'MarkerFaceColor', 'k')
    text(x(1, 1) + 0.08, x(2, 1), x(3, 1) + 0.05, 'P_i', 'FontSize', 12, 'FontWeight', 'bold')
    
    plot3(x(1, N), x(2, N), x(3, N), 'ko', 'MarkerFaceColor', 'k')
    text(x(1, N) + 0.08, x(2, N), x(3, N) + 0.10, 'P_f', 'FontSize', 12, 'FontWeight', 'bold')
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    end




% constructor
function [clientID, vrep ] = StartVrep(porta, Ts)
    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation
    
    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected,
    % and run matlab before v-rep otherwise it will not be changed
    vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, Ts, vrep.simx_opmode_oneshot_wait);
    vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_realtime_simulation, true, vrep.simx_opmode_oneshot_wait);
    vrep.simxSetBooleanParameter(clientID, vrep.sim_boolparam_dynamics_handling_enabled, false, vrep.simx_opmode_oneshot_wait);
    
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);

end  

% destructor
function DeleteVrep(clientID, vrep)
    
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
    %vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');
    
end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)
           
    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end
    
end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)
           
    [~,n] = size(q);
    
    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)   
           fprintf('failed to send the string signal of iteration %d \n',i); 
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high
    
end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)
    
    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n          
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)   
           fprintf(' failed to get position of joint %d \n',j); 
    end

end

