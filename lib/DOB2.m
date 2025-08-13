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
        Dist_0      % Disturbance estimate for step input (vector, same size as disturbance 'd')
        Dist_1      % Disturbance estimate for ramp input (vector, same size as disturbance 'd')
        Dist_2      % Disturbance estimate for parabolic input (vector, same size as disturbance 'd')
        sys_output  % Output of f(x, u, t) + F*d_hat, representing the estimated system dynamics without true disturbance (vector, same size as state derivative)
        d_hat       % Estimated disturbance (vector, size depends on the 'state' input to update_dist_estimate)
        F           % Transformation matrix for disturbance estimation, mapping disturbance to state space (matrix, size depends on system state and disturbance dimensions)
        pseudo_F    % Pseudo-inverse of the transformation matrix for disturbance estimation (matrix, size depends on system state and disturbance dimensions)
        z_dot       % Product of pseudo_F * sys_output, representing the derivative of the observer state 'z' (vector, same size as 'z')
        z           % State vector for the disturbance observer, used in the estimation process (vector, same size as 'd')
    end

    methods
        function obj = DOB2(DOB2_PROPERTIES)
            % DOB2 Constructor
            % Initializes the DOB2 object with provided properties.
            %
            % Inputs:
            %   - DOB2_PROPERTIES: A structure containing initialization parameters.
            %                      Expected fields:
            %                      .dt: Sampling time.
            %                      .gamma_0: Cut-off frequency for step disturbance.
            %                      .gamma_1: Cut-off frequency for ramp disturbance.
            %                      .gamma_2: Cut-off frequency for parabolic disturbance.
            %
            % Output:
            %   - obj: The initialized DOB2 object.
            %
            % Relationship between Inputs and Output:
            %   The input structure 'DOB2_PROPERTIES' directly populates the
            %   corresponding properties (dt, Gamma_0, Gamma_1, Gamma_2) of the
            %   newly created 'obj' instance. Other internal states (Dist_0, Dist_1,
            %   Dist_2, sys_output, d_hat, F, pseudo_F, z_dot) are initialized to
            %   zero or NaN, serving as starting points for the observer's operation.
            obj.dt = DOB2_PROPERTIES.dt;
            obj.Gamma_0 = DOB2_PROPERTIES.gamma_0;
            obj.Gamma_1 = DOB2_PROPERTIES.gamma_1;
            obj.Gamma_2 = DOB2_PROPERTIES.gamma_2;
            obj.Dist_0 = 0;
            obj.Dist_1 = 0;
            obj.Dist_2 = 0;
            obj.sys_output = 0;
            obj.d_hat = NaN; % Initialized to NaN to trigger initial zeros() assignment
            obj.F = 0;
            obj.pseudo_F = 0;
            obj.z_dot = 0;
            obj.z = 0; % Initializing z for the first integration step
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
                % Assumes the disturbance acts on the same dimension as the state.
                % This might need adjustment if disturbance dimension is different.
                obj.d_hat = zeros([3, 1]);
                obj.z = zeros([3, 1]); % Initialize z based on the expected size from pseudo_F * state
            end
            obj.update_ode_model(state, input, properties);
            obj.integ_z();
            obj.update_Dist_i(state);
            obj.d_hat = obj.Gamma_0 * obj.Dist_0 + obj.Gamma_1 * obj.Dist_1 + obj.Gamma_2 * obj.Dist_2;
        end

        function obj = update_ode_model(obj, state, input, properties)
            % update_ode_model: Computes the time derivatives of the state variables
            % (f(x,u,t)) and updates the disturbance observer's 'F' matrix and its
            % pseudo-inverse 'pseudo_F'. It also calculates 'sys_output', which
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
            %   - obj: The updated DOB2 object with 'F', 'pseudo_F', and 'sys_output' updated.
            
            mass = properties.mass;
            J = properties.J;
            D = properties.D;                       % Drag matrix/coefficient
            Vel = state(4:6);
            Quat = state(7:10)./norm(state(7:10));  % Normalize quaternion to ensure it's a unit quaternion
            Omg = state(11:13);

            % Quaternion to angular velocity transformation matrix
            quatOmega = [        0,  -Omg(1), -Omg(2), -Omg(3);...
                         Omg(1),        0,  Omg(3), -Omg(2);...
                         Omg(2), -Omg(3),        0,  Omg(1);...
                         Omg(3),  Omg(2), -Omg(1),        0];

            % Rotation matrix from Body frame to Inertial frame
            rotmB2I =[Quat(1)^2 + Quat(2)^2 - Quat(3)^2 - Quat(4)^2,        2*(Quat(2)*Quat(3) - Quat(1)*Quat(4)),          2*(Quat(2)*Quat(4) + Quat(1)*Quat(3));...
                      2*(Quat(2)*Quat(3) + Quat(1)*Quat(4)),  Quat(1)^2 - Quat(2)^2 + Quat(3)^2 - Quat(4)^2,          2*(Quat(3)*Quat(4) - Quat(1)*Quat(2));...
                      2*(Quat(2)*Quat(4) - Quat(1)*Quat(3)),        2*(Quat(3)*Quat(4) + Quat(1)*Quat(2)),  Quat(1)^2 - Quat(2)^2 - Quat(3)^2 + Quat(4)^2];

            Thrust = [0; 0; input(1)];                  % Thrust is along the z-axis in body frame
            Moment = [input(2); input(3); input(4);];   % Moments about body x, y, z axes

            % 'F' matrix construction: Relates disturbance to state derivatives.
            % Here, it's assumed disturbance primarily affects velocity (acceleration)
            % and potentially angular velocity. The drag term is included here.
            obj.F = [zeros([3, 3]);... 
                     -rotmB2I*D;...    
                     zeros([7, 3])];   
            % Note: The disturbance 'd' (which 'd_hat' estimates) is implicitly
            % assumed to be a 3-element vector related to forces/accelerations.
            % The exact physical meaning of 'd' depends on how 'F' is structured.

            obj.pseudo_F = pinv(obj.F); % Pseudo-inverse of F, used for observer gain

            % Nominal system dynamics (f(x, u, t)) without disturbance
            dpos = Vel;                                                             % Derivative of position is velocity
            dvel = [0; 0; 9.81] - rotmB2I*Thrust./mass - rotmB2I*D*rotmB2I'*Vel;    % Acceleration (gravity, thrust, drag)
            dquat = quatOmega*Quat./2;                                              % Derivative of quaternion
            domg = J\(-cross(Omg', (J*Omg)')' + Moment);                            % Derivative of angular velocity (Euler's equations)

            % Estimated system output (f(x,u,t) + F*d_hat)
            % This is the system dynamics predicted by the model plus the effect
            % of the *estimated* disturbance.
            obj.sys_output = [dpos; dvel; dquat; domg] + obj.F*obj.d_hat;
        end

        function obj = integ_z(obj)
            % integ_z: Integrates the observer state 'z' using Euler integration.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %
            % Output:
            %   - obj: The updated DOB2 object with the integrated 'z' state.
            %
            
            obj.z_dot = obj.pseudo_F * obj.sys_output;
            obj.z = obj.z + obj.dt * obj.z_dot;
        end

        function obj = update_Dist_i(obj, state)
            % update_Dist_i: Calculates intermediate disturbance estimates (Dist_0, Dist_1, Dist_2)
            % based on the difference between the actual system state (transformed by pseudo_F)
            % and the observer's internal state 'z'.
            %
            % Inputs:
            %   - obj: The DOB2 object itself.
            %   - state: The current state vector of the system.
            %
            % Output:
            %   - obj: The updated DOB2 object with new 'Dist_0', 'Dist_1', and 'Dist_2' values.
            
            obj.Dist_0 = obj.pseudo_F*state - obj.z;
            obj.Dist_1 = obj.Dist_0 + obj.Dist_0*obj.dt; % This appears to be a simple integration
            obj.Dist_2 = obj.Dist_1 + obj.Dist_1*obj.dt; % Another level of integration
        end
    end
end
