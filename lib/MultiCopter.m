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

        T       % Thrust
        Mx      % Moment around x-axis
        My      % Moment around y-axis
        Mz      % Moment around z-axis

        dt      % Time step
    end

    methods
        function obj = MultiCopter(initCond, initInput, inertialProperties, samplingTime)
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

            obj.att = obj.get_quat2eul();
            obj.rol = obj.att(1);
            obj.pit = obj.att(2);
            obj.yaw = obj.att(3);

            obj.p = initCond.omg(1);
            obj.q = initCond.omg(2);
            obj.r = initCond.omg(3);
            obj.omg = [obj.p; obj.q; obj.r];

            obj.T = initInput.T;
            obj.Mx = initInput.Mx;
            obj.My = initInput.My;
            obj.Mz = initInput.Mz;

            obj.dt = samplingTime;
        end

        function obj = update_states(obj)
            tspan = [0, obj.dt];

            initialState = [obj.pos; obj.vel; obj.quat; obj.omg];
            input = [obj.T; obj.Mx; obj.My; obj.Mz];
            % odeFunc = @(t, state) obj.ode_equations(t, state, input);

            % [~, state] = ode45(@(t, state) obj.ode_equations(t, state, input), tspan, initialState);
            [~, state] = ode15s(@(t, state) obj.ode_equations(t, state, input), tspan, initialState);

            obj.pos = state(end, 1:3)';
            obj.vel = state(end, 4:6)';
            obj.quat = state(end, 7:10)';
            obj.quat = obj.quat./norm(obj.quat);
            obj.att = obj.get_quat2eul();
            obj.omg = state(end, 11:13)';
            obj.set_state();
        end

        function dstate = ode_equations(obj, ~, state, input)
            % This function computes the time derivatives of the state variables
            % for the multi-copter dynamics. The state includes position, velocity,
            % quaternion orientation, and angular velocity. The input includes thrust
            % and moments applied to the multi-copter.
            %
            % Inputs:
            %   - state: A vector containing the current state of the multi-copter,
            %             which includes position (3), velocity (3), quaternion (4),
            %             and angular velocity (3).
            %   - input: A vector containing the control inputs, which includes thrust
            %             (T) and moments (Mx, My, Mz).
            %
            % Outputs:
            %   - dstate: A vector containing the time derivatives of the state variables,
            %              which includes the derivatives of position, velocity, quaternion,
            %              and angular velocity.

            % Pos = state(1:3);
            Vel = state(4:6);
            Quat = state(7:10);
            Omg = state(11:13);
            quatOmega = obj.get_skew_matrix();
            rotmB2I = obj.get_rotm_body2inertial();
            Thrust = [0; 0; input(1)];
            Moment = [input(2); input(3); input(4);];

            dpos = Vel;
            dvel = [0; 0; 9.81] - rotmB2I*Thrust./obj.mass;
            dquat = quatOmega*Quat./2;
            domg = obj.J\(-cross(Omg', (obj.J*Omg)')' + Moment);

            dstate = [dpos; dvel; dquat; domg];
        end

        function obj = set_input(obj, input)
            obj.T = input.T;
            obj.Mx = input.Mx;
            obj.My = input.My;
            obj.Mz = input.Mz;
        end

        function obj = set_state(obj)
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

        function rotm = get_rotm_body2inertial(obj)
            w = obj.quat(1);
            x = obj.quat(2);
            y = obj.quat(3);
            z = obj.quat(4);
            
            rotm = [w^2 + x^2 - y^2 - z^2,          2*(x*y - w*z),           2*(x*z + w*y);...
                            2*(x*y + w*z),  w^2 - x^2 + y^2 - z^2,           2*(y*z - w*x);...
                            2*(x*z - w*y),          2*(y*z + w*x),  w^2 - x^2 - y^2 + z^2];
        end
        
        function skew_matrix = get_skew_matrix(obj)
            skew_matrix = [         0, -obj.omg(1), -obj.omg(2), -obj.omg(3);...
                           obj.omg(1),           0,  obj.omg(3), -obj.omg(2);...
                           obj.omg(2), -obj.omg(3),           0,  obj.omg(1);...
                           obj.omg(3),  obj.omg(2), -obj.omg(1),          0];
        end

        function euler = get_quat2eul(obj)
            w = obj.quat(1);
            x = obj.quat(2);
            y = obj.quat(3);
            z = obj.quat(4);

            euler = NaN([3, 1]);
            euler(1) = atan2(2*(w*x + y*z), w^2 + z^2 - x^2 - y^2);
            euler(2) = asin(2*(w*y - x*z));
            euler(3) = atan2(2*(w*z + x*y), w^2 + x^2 - y^2 - z^2);
        end
    end
end


