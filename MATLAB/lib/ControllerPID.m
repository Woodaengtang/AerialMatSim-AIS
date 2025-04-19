classdef ControllerPID < handle
    % ControllerPID: A simple PID controller
    
    properties
        kp          % Proportional gain
        ki          % Integral gain
        kd          % Differential gain
        
        prevError   % Previous error (NaN until first update)
        intError    % Accumulated integral of error
        output      % Most recent controller output
        dt          % Sampling time
        u_min       % Minimum input saturation
        u_max       % Maximum input saturation
    end
    
    methods
        function obj = ControllerPID(kp, ki, kd, dt)
            % Constructor: set gains and sampling time, initialize state
            if dt <= 0
                error("Positive sampling time dt is required");
            end
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.dt = dt;
            
            % Initialize error history to NaN so first derivative term is forced to zero
            obj.prevError = NaN;
            obj.intError  = 0;
            obj.output    = 0;
            obj.u_min = Nan;
            obj.u_max = Nan;
        end
        
        function reset(obj)
            % reset: clears integral and derivative history
            obj.prevError = NaN;
            obj.intError  = 0;
            obj.output    = 0;
        end

        function set_output_saturation(obj, u_min, u_max)
            % set_input_saturation: setting the saturation of the controller output
            obj.u_min = u_min;
            obj.u_max = u_max;
        end

        function u = input_saturation(obj, u)
            % input_saturation: clamping the output to [u_min,u_max]
            if ~isnan(obj.u_min)
                u = max(u, obj.u_min);
            end
            if ~isnan(obj.u_max)
                u = min(u, obj.u_max);   % element‑wise min
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
            obj.intError = obj.intError + err.*obj.dt;
            I = obj.ki * obj.intError;
            
            % Derivative term: zero on first update to avoid kick
            if any(isnan(obj.prevError))    % Considering vector operation
                D = 0;
            else
                D = obj.kd.*(err - obj.prevError)./obj.dt;
            end
            
            unsat = P + I + D;
            obj.output = obj.input_saturation(obj, unsat); % Combine terms
            u = obj.output;
            
            obj.prevError = err;    % Save error for next derivative calculation
        end
    end
end
