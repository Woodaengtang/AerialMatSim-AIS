classdef SuccessivePID < handle
    % ControllerPID: A simple PID controller

    properties
        vel_loop                % Number of iterations for velocity control loop  
        att_loop                % Number of iterations for attitude control loop

        dt                      % Sampling time
        att_dt                  % Attitude loop time
        vel_dt                  % Velocity loop time

        p_des                   % Desired roll rate
        p_err                   % Current roll rate error
        p_err_prev              % Previous roll rate error
        p_err_sum               % Sum (integral) of roll rate error

        q_des                   % Desired pitch rate
        q_err                   % Current pitch rate error
        q_err_prev              % Previous pitch rate error
        q_err_sum               % Sum (integral) of pitch rate error

        r_des                   % Desired yaw rate
        r_err                   % Current yaw rate error
        r_err_prev              % Previous yaw rate error
        r_err_sum               % Sum (integral) of yaw rate error

        phi_des                 % Desired roll angle (phi)
        phi_err                 % Roll angle error
        phi_err_prev            % Previous roll angle error
        phi_err_sum             % Integral (sum) of roll angle error

        theta_des               % Desired pitch angle (theta)
        theta_err               % Pitch angle error
        theta_err_prev          % Previous pitch angle error
        theta_err_sum           % Integral (sum) of pitch angle error

        psi_des                 % Desired yaw angle (psi)
        psi_err                 % Yaw angle error
        psi_err_prev            % Previous yaw angle error
        psi_err_sum             % Integral (sum) of yaw angle error

        vn_des                  % Desired x velocity
        ve_des                  % Desired y velocity
        vd_des                  % Desired z velocity

        vx_err_body             % x velocity error in body frame
        vx_err_body_prev        % Previous x velocity error in body frame
        vx_err_body_sum         % Integral (sum) of x velocity error in body frame

        vy_err_body             % y velocity error in body frame
        vy_err_body_prev        % Previous y velocity error in body frame
        vy_err_body_sum         % Integral (sum) of y velocity error in body frame

        vz_err_body             % z velocity error in body frame
        vz_err_body_prev        % Previous z velocity error in body frame
        vz_err_body_sum         % Integral (sum) of z velocity error in body frame

        GainOmg                 % PID gains for angular rates
        GainAtt                 % PID gains for attitude control
        GainVel                 % PID gains for velocity control
        
        output                  % Current output value
        count                   % Counter for iterations
        zero_zdot_thrust        % Thrust value when vertical velocity is zero
    end

    methods
        function obj = SuccessivePID(sampling_time, Gains)
            obj.vel_loop = 20;
            obj.att_loop = 5;

            obj.dt = sampling_time;
            obj.att_dt = obj.att_loop*sampling_time;
            obj.vel_dt = obj.vel_loop*sampling_time;


            obj.p_des = 0;
            obj.p_err = 0;
            obj.p_err_prev = 0;
            obj.p_err_sum = 0;

            obj.q_des = 0;
            obj.q_err = 0;
            obj.q_err_prev = 0;
            obj.q_err_sum = 0;

            obj.r_des = 0;
            obj.r_err = 0;
            obj.r_err_prev = 0;
            obj.r_err_sum = 0;


            obj.phi_des = 0;
            obj.phi_err = 0;
            obj.phi_err_prev = 0;
            obj.phi_err_sum = 0;

            obj.theta_des = 0;
            obj.theta_err = 0;
            obj.theta_err_prev = 0;
            obj.theta_err_sum = 0;

            obj.psi_des = 0;
            obj.psi_err = 0;
            obj.psi_err_prev = 0;
            obj.psi_err_sum = 0;


            obj.vn_des = 0;
            obj.ve_des = 0;
            obj.vd_des = 0;

            obj.vx_err_body = 0;
            obj.vx_err_body_prev = 0;
            obj.vx_err_body_sum = 0;

            obj.vy_err_body = 0;
            obj.vy_err_body_prev = 0;
            obj.vy_err_body_sum = 0;

            obj.vz_err_body = 0;
            obj.vz_err_body_prev = 0;
            obj.vz_err_body_sum = 0;

            obj.GainOmg = struct('kp', Gains.omg_kp, 'ki', Gains.omg_ki, 'kd', Gains.omg_kd);
            obj.GainAtt = struct('kp', Gains.att_kp, 'ki', Gains.att_ki, 'kd', Gains.att_kd);
            obj.GainVel = struct('kp', Gains.vel_kp, 'ki', Gains.vel_ki, 'kd', Gains.vel_kd);

            obj.output = zeros([4, 1]);
            obj.count = 0;
            obj.zero_zdot_thrust = 19.62;
        end

        function output = update_PID(obj, state, guidance)
            % update_PID: Updates the PID controller based on current state and guidance.
            %             This function orchestrates the updates for velocity, attitude, and angular rate
            %             control loops, operating at different frequencies.
            %
            % Inputs:
            %   - obj: The SuccessivePID object.
            %   - state: The full state vector of the multi-copter.
            %   - guidance: Desired velocity guidance vector.
            %
            % Output:
            %   - output: The computed control outputs (Thrust, Mx, My, Mz).

            state_vel = state(4:6);
            state_att_quat = state(7:10);
            state_att_eul = quat2eul(state_att_quat', 'XYZ');
            state_omg = state(11:13);
            if mod(obj.count, obj.vel_loop) == 0
                obj.update_vel(state_vel, guidance, state_att_eul);
            end
            if mod(obj.count, obj.att_loop) == 0
                obj.update_att(state_att_eul);
            end
            obj.update_omg(state_omg);
            output = obj.output;
            obj.count = obj.count + 1;
        end

        function obj = update_vel(obj, state_vel, guidance, attitude)
            % update_vel: Updates the desired attitude (phi_des, theta_des, psi_des) based on velocity errors.
            %
            % Inputs:
            %   - obj: The SuccessivePID object.
            %   - state_vel: Current velocity vector [vn; ve; vd].
            %   - guidance: Desired velocity vector [vn_des; ve_des; vd_des].
            %   - attitude: Current Euler angles [roll; pitch; yaw].
            %
            % Output:
            %   - obj: The SuccessivePID object with updated desired attitude values.
            %          Specifically, obj.phi_des, obj.theta_des, and obj.psi_des are updated.
            
            % The eul2rotm function is not standard and was replaced by quat2rotm(eul2quat(...))
            rotm_b2i = quat2rotm(eul2quat(attitude, 'XYZ'));
            rotm_i2b = rotm_b2i';
            obj.vn_des  = guidance(1);
            obj.ve_des  = guidance(2);
            obj.vd_des  = guidance(3);
            
            vn_err = obj.vn_des - state_vel(1);
            ve_err = obj.ve_des - state_vel(2);
            vd_err = obj.vd_des - state_vel(3);

            error_vec = rotm_i2b*[vn_err; ve_err; vd_err];
            
            obj.vx_err_body = error_vec(1);
            obj.vy_err_body = error_vec(2);
            obj.vz_err_body = error_vec(3);

            obj.phi_des     = obj.GainVel.kp*obj.vy_err_body + obj.GainVel.ki*obj.vy_err_body_sum + obj.GainVel.kd*(obj.vy_err_body - obj.vy_err_body_prev)/obj.vel_dt;
            obj.phi_des     = deg2rad(obj.phi_des);
            obj.vy_err_body_prev = obj.vy_err_body;
            obj.vy_err_body_sum  = obj.vy_err_body_sum + obj.vy_err_body * obj.vel_dt;

            obj.theta_des   = -(obj.GainVel.kp*obj.vx_err_body + obj.GainVel.ki*obj.vx_err_body_sum + obj.GainVel.kd*(obj.vx_err_body - obj.vx_err_body_prev)/obj.vel_dt);
            obj.theta_des   = deg2rad(obj.theta_des);
            obj.vx_err_body_prev = obj.vx_err_body;
            obj.vx_err_body_sum  = obj.vx_err_body_sum + obj.vx_err_body * obj.vel_dt;

            obj.psi_des     = atan2(obj.ve_des, obj.vn_des);

            obj.output(1) = obj.zero_zdot_thrust - (obj.GainVel.kp*obj.vz_err_body + obj.GainVel.ki*obj.vz_err_body_sum + obj.GainVel.kd*(obj.vz_err_body - obj.vz_err_body_prev)/obj.vel_dt);
            obj.vz_err_body_prev = obj.vz_err_body;
            obj.vz_err_body_sum = obj.vz_err_body_sum + obj.vz_err_body * obj.vel_dt;
        end

        function obj = update_att(obj, state_att)
            % update_att: Updates the desired angular rates (p_des, q_des, r_des) based on Euler angle errors.
            %
            % Inputs:
            %   - obj: The SuccessivePID object.
            %   - state_att: Current Euler angles [roll; pitch; yaw].
            %
            % Output:
            %   - obj: The SuccessivePID object with updated desired angular rate values.
            %          Specifically, obj.p_des, obj.q_des, and obj.r_des are updated.

            obj.phi_err = obj.phi_des - state_att(1);
            obj.theta_err = obj.theta_des - state_att(2);
            obj.psi_err = obj.psi_des - state_att(3);

            obj.p_des = obj.GainAtt.kp * obj.phi_err + obj.GainAtt.ki * obj.phi_err_sum + obj.GainAtt.kd * (obj.phi_err - obj.phi_err_prev) / obj.att_dt;
            obj.phi_err_prev = obj.phi_err;
            obj.phi_err_sum = obj.phi_err_sum + obj.phi_err * obj.att_dt;

            obj.q_des = obj.GainAtt.kp * obj.theta_err + obj.GainAtt.ki * obj.theta_err_sum + obj.GainAtt.kd * (obj.theta_err - obj.theta_err_prev) / obj.att_dt;
            obj.theta_err_prev = obj.theta_err;
            obj.theta_err_sum = obj.theta_err_sum + obj.theta_err * obj.att_dt;

            obj.r_des = obj.GainAtt.kp * obj.psi_err + obj.GainAtt.ki * obj.psi_err_sum + obj.GainAtt.kd * (obj.psi_err - obj.psi_err_prev) / obj.att_dt;
            obj.psi_err_prev = obj.psi_err;
            obj.psi_err_sum = obj.psi_err_sum + obj.psi_err * obj.att_dt;
        end

        function obj = update_omg(obj, state_omg)
            % update_omg: Updates the desired angular rates based on current angular velocities.
            %
            % Inputs:
            %   - obj: The SuccessivePID object.
            %   - state_omg: Current angular velocities [p; q; r].
            %
            % Output:
            %   - obj: The SuccessivePID object with updated desired angular rate values.
            obj.p_err = obj.p_des - state_omg(1);
            obj.q_err = obj.q_des - state_omg(2);
            obj.r_err = obj.r_des - state_omg(3);

            obj.output(2) = obj.GainOmg.kp * obj.p_err + obj.GainOmg.ki * obj.p_err_sum + obj.GainOmg.kd * (obj.p_err - obj.p_err_prev) / obj.dt;
            obj.p_err_prev = obj.p_err;
            obj.p_err_sum = obj.p_err_sum + obj.p_err * obj.dt;

            obj.output(3) = obj.GainOmg.kp * obj.q_err + obj.GainOmg.ki * obj.q_err_sum + obj.GainOmg.kd * (obj.q_err - obj.q_err_prev) / obj.dt;
            obj.q_err_prev = obj.q_err;
            obj.q_err_sum = obj.q_err_sum + obj.q_err * obj.dt;

            obj.output(4) = obj.GainOmg.kp * obj.r_err + obj.GainOmg.ki * obj.r_err_sum + obj.GainOmg.kd * (obj.r_err - obj.r_err_prev) / obj.dt;
            obj.r_err_prev = obj.r_err;
            obj.r_err_sum = obj.r_err_sum + obj.r_err * obj.dt;
        end
        
        function command_vec = get_command(obj)
            % get_command: Returns the current control output vector.
            % This function aggregates various desired values (velocities, attitudes, angular rates)
            % that are computed by the successive PID loops.
            %
            % Output:
            %   - command_vec: A vector containing desired velocities (vn_des, ve_des, vd_des),
            %                  desired Euler angles (phi_des, theta_des, psi_des), and
            %                  desired angular rates (p_des, q_des, r_des).
            command_vec = [obj.vn_des; obj.ve_des; obj.vd_des; obj.phi_des; obj.theta_des; obj.psi_des; obj.p_des; obj.q_des; obj.r_des;];
        end

        % function obj = saturation_vel(obj)
            
        % end
    end
end
