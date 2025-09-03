classdef ControllerPID < handle
    % ControllerPID: A simple PID controller

    properties
        kp          % Proportional gain
        ki          % Integral gain
        kd          % Differential gain

        err_prev   % Previous error (NaN until first update)
        err_int    % Accumulated integral of error
        output      % Most recent controller output
        dt          % Sampling time
        u_min       % Minimum input saturation
        u_max       % Maximum input saturation
    end

    methods
        function obj = ControllerPID(kp, ki, kd, dt)
            % Constructor: set gains and sampling time, initialize state
            if (dt <= 0)
                error("Positive sampling time dt is required");

            end
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.dt = dt;
            % Initialize error history to NaN so first derivative term is forced to zero
            obj.err_prev = NaN;
            obj.err_int  = 0;
            obj.output    = 0;
            obj.u_min = NaN;
            obj.u_max = NaN;
        end

        function obj = reset(obj)
            % reset: clears integral and derivative history
            obj.err_prev = NaN;
            obj.err_int  = 0;
            obj.output    = 0;
        end

        function obj = set_output_saturation(obj, u_min, u_max)
            % set_input_saturation: setting the saturation of the controller output
            obj.u_min = u_min;
            obj.u_max = u_max;
        end

        function u = output_saturation(obj, u)
            % input_saturation: clamping the output to [u_min,u_max]
            if ~isnan(obj.u_min)
                u = max(u, obj.u_min);
            end
            if ~isnan(obj.u_max)
                u = min(u, obj.u_max);
            end
        end

        function u = update(obj, reference, measurement)
            % update: compute PID output, forcing D=0 on first call
            %
            % Inputs:
            %   setpoint    - desired target value
            %   measurement - current measured value
            % Output:
            %   u           - control output

            err = reference - measurement;

            % Proportional term
            P = obj.kp.*err;

            % Integral term
            I = obj.ki * obj.err_int;

            % Derivative term: zero on first update to avoid kick
            if any(isnan(obj.err_prev))    % Considering vector operation
                D = 0;
            else
                D = obj.kd.*(err - obj.err_prev)./obj.dt;
            end

            unsat = P + I + D;
            u = obj.output_saturation(unsat);

            % Anti-windup: only integrate if output is not saturated
            if u == unsat
                obj.err_int = obj.err_int + err.*obj.dt;
            end

            obj.output = u;

            obj.err_prev = err;
        end
    end
end
