classdef DOB2 < handle
    % High Order DOB (Disturbance Observer) DOB2 Class
    % This class is designed to implement a high order disturbance observer
    % which can estimate and compensate for disturbances in dynamic systems.

    % d/dt(x) = f(x, u, t) + Fd(t)
    properties
        dt          % Sampling time of controlled system
        Gamma_0     % Cut-off frequency of Step Disturbance (scalar gain)
        Gamma_1     % Cut-off frequency of Ramp Disturbance (scalar gain)
        Gamma_2     % Cut-off frequency of Parabolic Disturbance (scalar gain)
        K           % Gain used to adjust the difference between the estimated state and the measured state for improved accuracy
        Dist_0      % Disturbance estimate for step input (vector, same size as disturbance 'd')
        Dist_1      % Disturbance estimate for ramp input (vector, same size as disturbance 'd')
        x_dot
        x_hat_dot   % Output of f(x, u, t) + F*d_hat, representing the estimated system dynamics without true disturbance (vector, same size as state derivative)
        x_hat       % Estimated state vector of the system, representing the state after accounting for the estimated disturbances.
        d_hat       % Estimated disturbance (vector, size depends on the 'state' input to update_dist_estimate)
        F           % Transformation matrix for disturbance estimation, mapping disturbance to state space (matrix, size depends on system state and disturbance dimensions)
        pseudo_F    % Pseudo-inverse of the transformation matrix for disturbance estimation (matrix, size depends on system state and disturbance dimensions)
        z_dot       % Product of pseudo_F * x_hat_dot, representing the derivative of the observer state 'z' (vector, same size as 'z')
        z           % State vector for the disturbance observer, used in the estimation process (vector, same size as 'd')
    end

    methods
        function obj = DOB2(DOB2_PROPERTIES, initCond)
            % DOB2 Constructor
            % This constructor initializes a new instance of the DOB2 class
            % with the specified properties and initial conditions.
            %
            % Inputs:
            %   - DOB2_PROPERTIES: A structure containing initialization parameters.
            %                      Expected fields:
            %                      .dt: Sampling time.
            %                      .gamma_0: Cut-off frequency for step disturbance.
            %                      .gamma_1: Cut-off frequency for ramp disturbance.
            %                      .gamma_2: Cut-off frequency for parabolic disturbance.
            %
            %   - initCond: A structure containing the initial conditions for the
            %                state of the system, including position, velocity,
            %                quaternion, and angular velocity.
            %
            % Output:
            %   - obj: The initialized DOB2 object.
            %
            % The input structure 'DOB2_PROPERTIES' directly populates the
            % corresponding properties (dt, Gamma_0, Gamma_1, Gamma_2) of the
            % newly created 'obj' instance. Other internal states (Dist_0, Dist_1,
            % Dist_2, x_hat_dot, d_hat, F, pseudo_F, z_dot) are initialized to
            % zero or NaN, serving as starting points for the observer's operation.
            obj.dt = DOB2_PROPERTIES.dt;
            obj.Gamma_0 = DOB2_PROPERTIES.gamma_0;
            obj.Gamma_1 = DOB2_PROPERTIES.gamma_1;
            obj.K       = DOB2_PROPERTIES.K;
            obj.Dist_0 = zeros([3, 1]);
            obj.Dist_1 = zeros([3, 1]);
            obj.x_dot = zeros([13, 1]);
            obj.x_hat_dot = zeros([13, 1]);
            obj.x_hat = [initCond.pos; initCond.vel; initCond.quat; initCond.omg];
            obj.d_hat = NaN; % Initialized to NaN to trigger initial zeros() assignment
            obj.F = 0;
            obj.pseudo_F = 0;
            obj.z_dot = zeros([3, 1]);
            obj.z = zeros([3, 1]);
        end

        function obj = update_dist_estimate(obj, state, input, properties)
            % update_dist_estimate
            % Updates the disturbance estimate based on the current system state,
            % control input, and system properties. This is the main observer update step.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %   - state: Current state vector of the system (e.g., [pos; vel; quat; omg]).
            %   - input: Current control input vector applied to the system (e.g., [T; Mx; My; Mz]).
            %   - properties: Structure containing physical properties of the system
            %                 (e.g., mass, J, D).
            %
            % Output:
            %   - obj: The updated DOB2 object with new disturbance estimates.
            if isnan(obj.d_hat)
                % Initialize d_hat and z based on the size of the state vector
                obj.d_hat = zeros([3, 1]);
                obj.z_dot = zeros([3, 1]);
                obj.z = zeros([3, 1]);
            end
            obj.update_ode_model(state, input, properties);
            obj.integ_z();
            obj.update_Dist_i();
            obj.d_hat = obj.Gamma_0 * obj.Dist_0 + obj.Gamma_1 * obj.Dist_1;
        end

        function obj = ode_model(obj, state, input, properties)
            % ode_model: Computes the time derivatives of the state variables
            % and updates the disturbance observer's 'F' matrix and its pseudo-inverse 'pseudo_F'.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %   - state: A vector containing the current state [pos; vel; quat; omg].
            %   - input: A vector containing the control inputs [T; Mx; My; Mz].
            %   - properties: A structure containing physical properties (mass, J, D).
            %
            % Output:
            %   - obj: The updated DOB2 object with 'x_dot', 'F', and 'pseudo_F' updated.
            
            mass = properties.mass;
            J = properties.J;
            D = properties.D;                       % Drag matrix/coefficient
            Vel = state(4:6);
            Quat = state(7:10)./norm(state(7:10));  % Normalize quaternion to ensure it's a unit quaternion
            Omg = state(11:13);

            % Quaternion to angular velocity transformation matrix
            p = Omg(1); q = Omg(2); r = Omg(3);
            quatOmega = [    0, -p, -q, -r;...
                p,  0,  r, -q;...
                q, -r,  0,  p;...
                r,  q, -p,  0];

            % Rotation matrix from Body frame to Inertial frame
            rotmB2I = quat2rotm(Quat');
            Thrust = [0; 0; input(1)];                                              % Thrust is along the z-axis in body frame
            Moment = [input(2); input(3); input(4)];                                % Moments about body x, y, z axes
            dpos = Vel;                                                             % Derivative of position is velocity
            dvel = [0; 0; 9.81] - rotmB2I*Thrust./mass - rotmB2I*D*rotmB2I'*Vel;    % Acceleration (gravity, thrust, drag)
            dquat = quatOmega*Quat./2;                                              % Derivative of quaternion
            domg = J\(Moment - cross(Omg', (J*Omg)')');                             % Derivative of angular velocity (Euler's equations)
            obj.x_dot = [dpos; dvel; dquat; domg];
            % 'F' matrix construction: Relates disturbance to state derivatives.
            obj.F = [zeros([3, 3]);... 
                     eye(3);...    
                     zeros([7, 3])];
            
            obj.pseudo_F = (obj.F'*obj.F)\obj.F';
        end

        function obj = update_ode_model(obj, state, input, properties)
            % update_ode_model: Computes the time derivatives of the state variables
            % (f(x,u,t)) and updates the disturbance observer's 'F' matrix and its
            % pseudo-inverse 'pseudo_F'. It also calculates 'x_hat_dot', which
            % is the estimated system dynamics without the true disturbance, but
            % including the effect of the *estimated* disturbance.
            %
            % Inputs:
            %   - obj: The MultiCopter object (though context suggests it's the DOB2 object).
            %   - state: A vector containing the current state [pos; vel; quat; omg].
            %   - input: A vector containing the control inputs [T; Mx; My; Mz].
            %   - properties: A structure containing physical properties (mass, J, D).
            %
            % Output:
            %   - obj: The updated DOB2 object with 'F', 'pseudo_F', and 'x_hat_dot' updated.
            
            obj.ode_model(state, input, properties);

            % Estimated system output (f(x,u,t) + F*d_hat + K(x_m - x_hat))
            % This is the system dynamics predicted by the model plus the effect
            % of the *estimated* disturbance.
            obj.x_hat_dot = obj.x_dot + obj.F*obj.d_hat + obj.K*(state - obj.x_hat);
        end

        function obj = integ_z(obj)
            % integ_z: Integrates the observer state 'z' using Euler integration.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %
            % Output:
            %   - obj: The updated DOB2 object with the integrated 'z' state.

            obj.z_dot = obj.pseudo_F * obj.x_dot + obj.d_hat;
            obj.z = obj.z + obj.dt * obj.z_dot;
            obj.x_hat = obj.x_hat + obj.dt * obj.x_hat_dot;
            obj.x_hat(7:10) = obj.x_hat(7:10)./norm(obj.x_hat(7:10));
        end

        % function obj = update_Dist_i(obj, state)
        function obj = update_Dist_i(obj)
            % update_Dist_i: Calculates intermediate disturba   nce estimates (Dist_0, Dist_1)
            % based on the difference between the actual system state (transformed by pseudo_F)
            % and the observer's internal state 'z'.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %   - state: The current state vector of the system.
            %
            % Output:
            %   - obj: The updated DOB2 object with new 'Dist_0', 'Dist_1', and 'Dist_2' values.
            
            
            obj.Dist_0 = obj.pseudo_F*obj.x_hat - obj.z;
            obj.Dist_1 = obj.Dist_1 + obj.Dist_0*obj.dt;
        end


        function state_vec = get_state(obj)
            % get_state_quat: Retrieves the estimated state of the multi-copter.
            %
            % Output:
            %   - state_vec: A vector containing the position, velocity, 
            %     attitude (quaternion), and angular velocity of the multi-copter.

            state_vec = obj.x_hat;
        end
    end
end

