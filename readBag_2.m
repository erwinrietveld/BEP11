% @author: Jon Gonzales
% @edited by: Thijs van der Burg
% Requirements: 
%   * Robotics System Toolbox
%   * Custom message support

% To install custon message support
% link:     http://www.mathworks.com/matlabcentral/fileexchange/49810-robotics-system-toolbox-interface-for-ros-custom-messages
% examples: http://www.mathworks.com/help/robotics/examples/work-with-specialized-ros-messages.html

%% Install custom barc messages
% Notes: 
%   * Uncomment the following commands to install custom messages and
%       replace 'path_to_msggen' with your system path to that folder
%   * Restart MATLAB after this step for the changes to take effect 
%   * If you add / delete / modify msg types, delete the matlabgen folder
%       within the packages folder

% rosgenmsg('packages')     % follow the output instructions
                            % file may be created if non-existent
% addpath('path_to_msggen')
% savepath('path_to_msggen')

%% select bag file
[fileName,pathName] = uigetfile({'data/*.bag'},'Select bag file');
filePath = strcat(pathName,fileName);

%%  Open rosbag 
bag     = rosbag(filePath);    % open bag file
msgs    = readMessages(bag);                    % get all messages
t0      = bag.StartTime;                        % get initial time
bag.AvailableTopics                             % display all recorded topics

%% extract messages
% get individual topic bags
bag_enc     = select(bag,'Topic','/encoder');
bag_imu     = select(bag,'Topic','/imu');
bag_ecu     = select(bag,'Topic','/rc_inputs');
bag_est     = select(bag,'Topic','/state_estimate');


% all message data
imu             = readMessages(bag_imu);
encoder         = readMessages(bag_enc);
ecu             = readMessages(bag_ecu);
est             = readMessages(bag_est);

% number of messages
n_imu_msg       = size(imu,1);
n_enc_msg       = size(encoder,1);
n_ecu_msg       = size(ecu,1);
n_est_msg       = size(est,1);

%% process imu data

if n_imu_msg > 0
    t_imu           = bag_imu.timeseries.Time - t0;     % time series
    f_imu           = 1/mean(diff(t_imu));              % average sampling frequency
    time_stamp      = zeros(1,n_imu_msg);
    roll            = zeros(1,n_imu_msg);
    pitch           = zeros(1,n_imu_msg);
    yaw             = zeros(1,n_imu_msg);
    X_acc           = zeros(1,n_imu_msg);
    Y_acc           = zeros(1,n_imu_msg);
    Z_acc           = zeros(1,n_imu_msg);
    rollrate        = zeros(1,n_imu_msg);
    pitchrate       = zeros(1,n_imu_msg);
    yawrate         = zeros(1,n_imu_msg);
    for i = 1:n_imu_msg
        time_stamp(i)   = imu{i}.Timestamp;
        roll(i)     = imu{i}.Value(1);
        pitch(i)    = imu{i}.Value(2);
        yaw(i)      = imu{i}.Value(3);
        X_acc(i)    = imu{i}.Value(4);
        Y_acc(i)    = imu{i}.Value(5);
        Z_acc(i)    = imu{i}.Value(6);
        rollrate(i) = imu{i}.Value(7);
        pitchrate(i)= imu{i}.Value(8);
        yawrate(i)  = imu{i}.Value(9);
    end
    %plot(t_imu, time_stamp);
    plot(t_imu, roll);
    xlabel('time [s]');
    ylabel('roll [rad]');
    figure
    plot(t_imu, pitch);
    xlabel('time [s]');
    ylabel('pitch [rad]');
    figure
    plot(t_imu, yaw);
    xlabel('time [s]');
    ylabel('yaw [rad]');
    figure
    plot(t_imu, X_acc);
    xlabel('time [s]');
    ylabel('X Acceleration [m/s^2]');
    figure
    plot(t_imu, Y_acc);
    xlabel('time [s]');
    ylabel('Y Acceleration [m/s^2]');
    figure
    plot(t_imu, Z_acc);
    xlabel('time [s]');
    ylabel('Z Acceleration [m/s^2]');
    figure
    plot(t_imu, rollrate);
    xlabel('time [s]');
    ylabel('roll rate [rad/s]');
    figure
    plot(t_imu, pitchrate);
    xlabel('time [s]');
    ylabel('pitch rate [rad/s]');
    figure
    plot(t_imu, yawrate);
    xlabel('time [s]');
    ylabel('yaw rate [rad/s]');
end

%% process encoder data
if n_enc_msg > 0
    t_enc   = bag_enc.timeseries.Time - t0;     % time series
    f_enc   = 1/mean(diff(t_enc));              % average sampling frequency
    n_FL    = zeros(1, n_enc_msg);
    n_FR    = zeros(1, n_enc_msg);
    n_BL    = zeros(1, n_enc_msg);
    n_BR    = zeros(1, n_enc_msg);
    for i=1:n_enc_msg
        n_FL(i)  = encoder{i}.FL;
        n_FR(i)  = encoder{i}.FR;
        n_BL(i)  = encoder{i}.BL;
        n_BR(i)  = encoder{i}.BR;
    end
    figure
    plot(t_enc, n_FL); hold on;
    plot(t_enc, n_FR); hold on;
    plot(t_enc, n_BL); hold on;
    plot(t_enc, n_BR); grid on;
    legend('FL','FR','BL','BR','Location','NorthWest');
    xlabel('time [s]');
    ylabel('Wheel revolutions [1/4th of a rotation]');
end

%% process ecu data
if n_ecu_msg > 0
    t_ecu       = bag_ecu.timeseries.Time - t0;     % time series
    f_ecu       = 1/mean(diff(t_ecu));              % average sampling frequency
    u_motor_pwm = zeros(1,n_ecu_msg);
    u_servo_pwm = zeros(1, n_ecu_msg);
    for i=1:n_ecu_msg
        u_motor_pwm(i)  = ecu{i}.MotorPwm;
        u_servo_pwm(i)  = ecu{i}.ServoPwm;
    end
    figure
    plot(t_ecu, smooth(u_motor_pwm),'LineWidth',1); hold on;
    %plot(t_ecu, u_servo_pwm,'LineWidth',1); grid on;
    %legend('Motor PWM','Servo PWM','Location','NorthWest');
    xlabel('time [s]');
    ylabel('Motor PWM signal'); grid on
end
%% process state estimation data
%{
if n_est_msg > 0
    t_est   = bag_est.timeseries.Time - t0;     % time series
    f_est   = 1/mean(diff(t_est));              % average sampling frequency
    n_X     = zeros(1, n_est_msg);
    n_Y     = zeros(1, n_est_msg);
    n_PSI   = zeros(1, n_est_msg);
    n_V     = zeros(1, n_est_msg);
    for i=1:n_est_msg
        n_X(i)      =   est{i}.X;
        n_Y(i)      =   est{i}.Y;
        n_PSI(i)    =   est{i}.Psi;
        n_V(i)      =   est{i}.V;
    end
    figure;     plot(t_est, n_X);       grid on;    xlabel('time [s]'); ylabel('X [m]');
    figure;     plot(t_est, n_Y);       grid on;    xlabel('time [s]'); ylabel('Y [m]');
    figure;     plot(t_est, n_PSI);     grid on;    xlabel('time [s]'); ylabel('PSI [rad]');
    figure;     plot(t_est, n_V);       grid on;    xlabel('time [s]'); ylabel('V [m/s]');
end
%}

%% plots and subplots
% Here you can make more plots from the data extracted above

%%
display(sprintf('avg imu sampling frequency \t: %f \t[Hz]', f_imu));
display(sprintf('avg enc sampling frequency \t: %f \t[Hz]', f_enc)); 
display(sprintf('avg ecu command frequency \t: %f \t[Hz]', f_ecu));
%display(sprintf('avg est command frequency \t: %f \t[Hz]', f_est));