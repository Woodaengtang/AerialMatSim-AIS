classdef MultiCopter < handle

    properties
        mass    % Mass of the multi-copter
        Jxx     % Moment of inertia around x-axis
        Jyy     % Moment of inertia around y-axis
        Jzz     % Moment of inertia around z-axis
        Jxy     % Product of inertia around x and y axes
        Jxz     % Product of inertia around x and z axes
        Jyz     % Product of inertia around y and z axes
        J       % Inertia matrix (Jxx, Jyy, Jzz, Jxy, Jxz, Jyz)

        pos     % Current position
        pn      % North position component
        pe      % East position component
        pd      % Down position component

        vel     % Current velocity
        vn      % North velocity component
        ve      % East velocity component
        vd      % Down velocity component

        quat    % Quaternion representation of orientation
        e0      % Scalar part of quaternion
        ex      % First component of quaternion
        ey      % Second component of quaternion
        ez      % Third component of quaternion

        att     % Current attitude (roll, pitch, yaw)
        rol     % Roll angle
        pit     % Pitch angle
        yaw     % Yaw angle

        omg     % Current angular velocity
        p       % Roll rate
        q       % Pitch rate
        r       % Yaw rate

        D       % Drag Coefficient
        vw      % Wind velocity in the body frame

        T       % Thrust
        Mx      % Moment around x-axis
        My      % Moment around y-axis
        Mz      % Moment around z-axis

        dt      % Time step
    end

    methods
        function obj = MultiCopter(initCond, initInput, inertialProperties, samplingTime)
            % MultiCopter constructor: Initializes the multi-copter object.
            %
            % Inputs:
            %   - initCond: Structure with initial conditions (pos, vel, quat, omg).
            %   - initInput: Structure with initial inputs (T, Mx, My, Mz).
            %   - inertialProperties: Structure with inertial properties (mass, Jxx, etc.).
            %   - samplingTime: The sampling time (dt) for the simulation.
            %
            % Output:
            %   - obj: The initialized MultiCopter object.

            obj.mass = inertialProperties.mass;
            obj.Jxx = inertialProperties.Jxx;
            obj.Jyy = inertialProperties.Jyy;
            obj.Jzz = inertialProperties.Jzz;
            obj.Jxy = inertialProperties.Jxy;
            obj.Jxz = inertialProperties.Jxz;
            obj.Jyz = inertialProperties.Jyz;
            obj.J = [obj.Jxx, obj.Jxy, obj.Jxz;...
                obj.Jxy, obj.Jyy, obj.Jyz;...
                obj.Jxz, obj.Jyz, obj.Jzz];

            obj.pn = initCond.pos(1);
            obj.pe = initCond.pos(2);
            obj.pd = initCond.pos(3);
            obj.pos = [obj.pn, obj.pe, obj.pd]';

            obj.vn = initCond.vel(1);
            obj.ve = initCond.vel(2);
            obj.vd = initCond.vel(3);
            obj.vel = [obj.vn, obj.ve, obj.vd]';

            obj.e0 = initCond.quat(1);
            obj.ex = initCond.quat(2);
            obj.ey = initCond.quat(3);
            obj.ez = initCond.quat(4);
            obj.quat = [obj.e0, obj.ex, obj.ey, obj.ez]';

            obj.att = quat2eul(obj.quat', 'XYZ')';
            obj.rol = obj.att(1);
            obj.pit = obj.att(2);
            obj.yaw = obj.att(3);

            obj.p = initCond.omg(1);
            obj.q = initCond.omg(2);
            obj.r = initCond.omg(3);
            obj.omg = [obj.p; obj.q; obj.r];

            obj.D = [0.1,   0, 0;...
                0, 0.1, 0;...
                0,   0, 0.001];
            obj.vw = zeros([3, 1]);

            obj.T = initInput.T;
            obj.Mx = initInput.Mx;
            obj.My = initInput.My;
            obj.Mz = initInput.Mz;

            obj.dt = samplingTime;
        end

        function obj = update_states(obj)
            % update_states: Updates the multi-copter's states over one time step.
            %
            % Input:
            %   - obj: The MultiCopter object.
            %
            % Output:
            %   - obj: The MultiCopter object with updated states.

            initialState = [obj.pos; obj.vel; obj.quat; obj.omg];
            input = [obj.T; obj.Mx; obj.My; obj.Mz];
            % odeFunc = @(t, state) obj.ode_equations(t, state, input);

            % Define the number of steps for the Runge-Kutta method
            numSteps = 10;
            dtRK = obj.dt / numSteps;
            state = initialState;

            % Wind disturbance in the body frame
            wind_dist = obj.vw;

            for i = 1:numSteps
                k1 = obj.ode_equations(state, input, wind_dist);
                k2 = obj.ode_equations(state + 0.5 * dtRK * k1, input, wind_dist);
                k3 = obj.ode_equations(state + 0.5 * dtRK * k2, input, wind_dist);
                k4 = obj.ode_equations(state + dtRK * k3, input, wind_dist);
                state = state + (dtRK / 6) * (k1 + 2*k2 + 2*k3 + k4);
            end

            obj.pos = state(1:3);
            obj.vel = state(4:6);
            obj.quat = state(7:10)./norm(state(7:10));
            obj.att = quat2eul(obj.quat', 'XYZ')';
            obj.omg = state(11:13);
            obj.set_state();
        end

        function dstate = ode_equations(obj, state, input, wind_dist)

            Vel = state(4:6);
            % 수정: 정규화를 제거하고, RK4에서 넘어온 '그대로의' 쿼터니언을 사용합니다.
            Quat = state(7:10);
            Omg = state(11:13);

            % Rotation matrix from body to inertial using Quat
            w = Quat(1); x = Quat(2); y = Quat(3); z = Quat(4);
            rotmB2I = [w^2 + x^2 - y^2 - z^2,          2*(x*y - w*z),           2*(x*z + w*y);...
                2*(x*y + w*z),  w^2 - x^2 + y^2 - z^2,           2*(y*z - w*x);...
                2*(x*z - w*y),          2*(y*z + w*x), w^2 - x^2 - y^2 + z^2];

            % Quaternion kinematic matrix using Omg = [p; q; r]
            p = Omg(1); q = Omg(2); r = Omg(3);
            quatOmega = [    0, -p, -q, -r;...
                p,  0,  r, -q;...
                q, -r,  0,  p;...
                r,  q, -p,  0];

            Thrust = [0; 0; input(1)];
            Moment = [input(2); input(3); input(4)];

            dpos = Vel;
            dvel = [0; 0; 9.81] - rotmB2I*Thrust./obj.mass - rotmB2I*obj.D*(rotmB2I'*Vel - wind_dist);
            dquat = quatOmega*Quat./2;
            domg = obj.J\(Moment - cross(Omg', (obj.J*Omg)')');

            dstate = [dpos; dvel; dquat; domg];
        end

        function obj = set_input(obj, input)
            % set_input: Sets the control inputs for the multi-copter.
            %
            % Inputs:
            %   - obj: The MultiCopter object.
            %   - input: Structure with control inputs (T, Mx, My, Mz).
            %
            % Output:
            %   - obj: The MultiCopter object with updated inputs.

            obj.T = input.T;
            obj.Mx = input.Mx;
            obj.My = input.My;
            obj.Mz = input.Mz;
        end

        function obj = set_state(obj)
            % set_state: Updates individual state properties from state vectors.
            %
            % Input:
            %   - obj: The MultiCopter object.
            %
            % Output:
            %   - obj: The MultiCopter object with updated individual properties.

            obj.pn = obj.pos(1);
            obj.pe = obj.pos(2);
            obj.pd = obj.pos(3);

            obj.vn = obj.vel(1);
            obj.ve = obj.vel(2);
            obj.vd = obj.vel(3);

            obj.e0 = obj.quat(1);
            obj.ex = obj.quat(2);
            obj.ey = obj.quat(3);
            obj.ez = obj.quat(4);

            obj.rol = obj.att(1);
            obj.pit = obj.att(2);
            obj.yaw = obj.att(3);

            obj.p = obj.omg(1);
            obj.q = obj.omg(2);
            obj.r = obj.omg(3);
        end

        function obj = set_body_wind(obj, body_wind)
            % set_body_wind: Sets the wind velocity in the body frame.
            %
            % Inputs:
            %   - obj: The MultiCopter object.
            %   - body_wind: The wind velocity vector in the body frame.
            %
            % Output:
            %   - obj: The MultiCopter object with updated wind velocity.

            obj.vw = body_wind;
        end

        function state_vec = get_state(obj)
            
            % get_state: Retrieves the current state of the multi-copter.
            %
            % Output:
            %   - state_vec: A vector containing the position, velocity, 
            %     attitude (Euler angles), and angular velocity of the multi-copter.
            state_vec = [obj.pos; obj.vel; obj.att; obj.omg];
        end
    end
end


