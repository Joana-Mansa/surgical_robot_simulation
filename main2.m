
function [t, q, q_act] = main2
    
    % main2
    % cleans workspace
    clear
    close all
    clc
    
    addpath('matlab/');
    addpath('functions_matlab/');
    porta = 19997; % default CoppeliaSim port

    % initialise robot variables
    init
    
    % Define the total duration and time step
    tf = 14; % in seconds (2s per side + 1s stop at each corner + 2s stop at initial position)
    Ts = 1e-3; % in seconds
    % time vector from 0 to tf
    t = 0 : Ts : tf;
    % number of time steps
    N = length(t);
    
    % dimension of task space
    % space in which the robot performs its tasks
    % related to the Cartesian coordinate frame
    n = 7;
    % joint position
    % Initial joint configuration (in degrees)
    q = zeros(n, N); 
    q(:, 1) = [48 23 -21 94 8 73 70]'/180*pi;
    dq  = zeros(n,N);

    % Joint variables in Jaco2 7DOFs convention
    q_jaco = zeros(7,N);
    q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));
    
    % quaternion orientation variables
    quat = zeros(4,N);
    quat_d = zeros(4, N);
    for i = 1:N
     quat_d(:, i) = [1, 0, 0, 0]'; % constant desired orientation ex ey ex eta
    end

    % error variables
    error_pos = zeros(3, N);  % Position error
    error_quat = zeros(3, N); % Quaternion error (3-element representation)
    error = zeros(6, N);  % Combined error

    % matrix condition number 
    condJ = zeros(1, N);  % Condition number of the Jacobian
    
    % Position variables
    x   = zeros(3,N); % current position
    posd = zeros(3,N); % desired position
    dposd = zeros(3,N); % desired velocity    


    % Denavit–Hartenberg parameters definition
    a     = zeros(7,1);
    alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
    d     = [0.2755 0 -0.410 -0.0098 -0.3072 0 0.25]';
    theta = q(:,1);
    DH = [a alpha d theta];

    % initial pos0 and final posf positions definition
    DH(:,4) = q(:,1);
    T = DirectKinematics(DH);
    % Define the initial and final positions
    pos0 = T(1:3, 4, n); % Assuming this is the initial position

    %curvilinear abscissa
    s_t = zeros(N,1);
    ds_t = zeros(N,1);
    dds_t = zeros(N,1);
    s0 = 0;
    s1 = 1;
    dsc = 0.6;


    % posf = pos0; % The robot returns to the initial position
    
    % Starting communication with CoppeliaSim
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta,Ts);    
    handle_joint = my_get_handle_Joint(vrep, clientID);      % handle to the joints
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,1)); % first move to q0

    % Corner points of the rectangle
    p1 = [-0.2, -0.4, 0.3]';
    p2 = [-0.2, -0.6, 0.3]';
    p3 = [0.2, -0.6, 0.3]';
    p4 = [0.2, -0.4, 0.3]';
    

% Select the algorithm
algorithm = 'inverse';
if strcmp(algorithm, 'inverse')
    K = 1*diag([10 10 10 10 10 10]);
    fprintf('\n algorithm: inverse of the jacobian \n');
 else
    K = 10*diag([9 9 12 12 12 12]);
    fprintf('\n algorithm: transpose of the jacobian \n');
end


% Inverse kinematics loop
for i = 1:N
    tic;

        if t(i) < 2
            [s_t, ds_t, ~] = trapezoidal(0, 1, 0.6, 2, t(i));
            posd(:, i) = p1 + (p2 - p1) * s_t;
            dposd(:, i) = (p2 - p1) * ds_t;
        elseif t(i) < 3
            % Stop at p2 for 1 second
            posd(:, i) = p2;
            dposd(:, i) = zeros(3,1);
        elseif t(i) < 5
            [s_t, ds_t, ~] = trapezoidal(0, 1, 0.6, 2, t(i) - 3);
            posd(:, i) = p2 + (p3 - p2) * s_t;
            dposd(:, i) = (p3 - p2) * ds_t;
        elseif t(i) < 6
            % Stop at p3 for 1 second
            posd(:, i) = p3;
            dposd(:, i) = zeros(3,1);
        elseif t(i) < 8
            [s_t, ds_t, ~] = trapezoidal(0, 1, 0.6, 2, t(i) - 6);
            posd(:, i) = p3 + (p4 - p3) * s_t;
            dposd(:, i) = (p4 - p3) * ds_t;
        elseif t(i) < 9
            % Stop at p4 for 1 second
            posd(:, i) = p4;
            dposd(:, i) = zeros(3,1);
        elseif t(i) < 11
            [s_t, ds_t, ~] = trapezoidal(0, 1, 0.6, 2, t(i) - 9);
            posd(:, i) = p4 + (p1 - p4) * s_t;
            dposd(:, i) = (p1 - p4) * ds_t;
        elseif t(i) < 13
            % Stop at p1 for 2 seconds
            posd(:, i) = p1;
            dposd(:, i) = zeros(3,1);
        else
            % Maintain final stop at p1
            posd(:, i) = p1;
            dposd(:, i) = zeros(3,1);
        end
    

    % Direct kinematics
    DH(:, 4) = q(:, i);
    T = DirectKinematics(DH);
    x(:, i) = T(1:3, 4, end);
    quat(:, i) = Rot2Quat(T(1:3, 1:3, end));

    % Jacobian
    J = Jacobian(DH);

    % Condition of the Jacobian is calculated and stored
    condJ(i) = cond(J);

    % Inverse kinematics algorithm
    error_pos(:, i) = posd(:, i) - x(:, i);
    error_quat(:, i) = QuatError(quat_d(:, i), quat(:, i));
    error(:, i) = [error_pos(:, i); error_quat(:, i)];

    % Inverse kinematics algorithm
    if strcmp(algorithm, 'transpose')
        dq(:, i) = J' * K * error(:, i);
    else
        dq(:, i) = pinv(J) * K * error(:, i);
    end

    % Euler integration
    if i < N
        q(:, i + 1) = q(:, i) + Ts * dq(:, i);
    end

    % Joint velocities for the Jaco Model in CoppeliaSim
    q_jaco(:, i) = mask_q_DH2Jaco(q(:, i));
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:, i));

    % Elapsed time from the iteration
    elapsedTime = toc; 
    pause(Ts - elapsedTime);
end

    % Finish simulation
    DeleteVrep(clientID, vrep); 

    % Plot the results
    figure;
    subplot(231);
    plot(t, q);
    ylabel('q [rad]');
    xlabel('time [s]');
    grid on;
    title('Joint position');

    subplot(234);
    plot(t, dq);
    ylabel('dq [rad/s]');
    xlabel('time [s]');
    title('Joint Velocity profile');

    subplot(232);
    plot(t, error(1:3, :));
    ylabel('p err [m]');
    xlabel('time [s]');
    grid on;
    title('Position Error');

    subplot(235);
    plot(t, error(4:6, :));
    ylabel('o err [-]');
    xlabel('time [s]');
    grid on;
    title('Orientation error');

    subplot(233);
    plot(t, condJ);
    ylabel('cond J');
    xlabel('time [s]');
    title('Condition Number of Jacobian');
    
    figure
    hold on
    DH(:, 4) = q(:, 1);
    DrawRobot(DH);
    DH(:, 4) = q(:, N);
    DrawRobot(DH);
    plot3(x(1, :), x(2, :), x(3, :), 'LineWidth',2.0)
    
    % makes sure the grid appears
    grid on
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    
    % Plot all corner points
    corner_points = [p1, p2, p3, p4];
    plot3(corner_points(1, :), corner_points(2, :), corner_points(3, :), 'ko', 'MarkerFaceColor', 'k');
    
    % Annotate corner points
    text_offset = 0.05;
    text(p1(1) + text_offset, p1(2), p1(3) + 0.1, 'p_1', 'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    text(p2(1) + text_offset, p2(2), p2(3) + 0.09, 'p_2','FontSize', 12, 'HorizontalAlignment',  'center', 'VerticalAlignment', 'top');
    text(p3(1) + text_offset, p3(2), p3(3) + 0.09, 'p_3', 'FontSize', 12,'HorizontalAlignment',  'center', 'VerticalAlignment', 'top');
    text(p4(1) + text_offset, p4(2), p4(3) + 0.09, 'p_4', 'FontSize', 12, 'HorizontalAlignment',  'center', 'VerticalAlignment', 'top');
   
    hold off
    
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
     