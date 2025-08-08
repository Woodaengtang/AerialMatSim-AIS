classdef DOB2
    % High Order DOB (Disturbance Observer) DOB2 Class
    % This class is designed to implement a high order disturbance observer
    % which can estimate and compensate for disturbances in dynamic systems.

    % d/dt(x) = f(x, u, t) + Fd(t)
    properties
        Gamma_0     % Cut-off frequency of Step Disturbance
        Gamma_1     % Cut-off frequency of Ramp Disturbance
        Gamma_2     % Cut-off frequency of Parabolic Disturbance
        Dist_0      % Disturbance estimate for step input
        Dist_1      % Disturbance estimate for ramp input
        Dist_2      % Disturbance estimate for parabolic input
        sys_output  % Output of f(x, u, t)
        d_hat       % Estimated disturbance
        dt          % Sampling time of controlled system
        pseudo_F    % Pseudo-inverse of the transformation matrix for disturbance estimation
        z_dot       % Product of pseudo_F * system dynamics
    end

    methods
        function obj = DOB2(DOB2_PROPERTIES)
            obj.Gamma_0 = DOB2_PROPERTIES.gamma_0;
            obj.Gamma_1 = DOB2_PROPERTIES.gamma_1;
            obj.Gamma_2 = DOB2_PROPERTIES.gamma_2;
            obj.Dist_0 = 0;
            obj.Dist_1 = 0;
            obj.Dist_2 = 0;
            obj.sys_output = 0;
            obj.d_hat = 0;
            obj.dt = DOB2_PROPERTIES.dt;
            obj.pseudo_F = DOB2_PROPERTIES.pseudo_F;
            obj.z_dot = 0;
        end

        function obj = get_dyn_model_output(obj, output)
            obj.sys_output = output;
        end

        function obj = integ_dist(obj)
            
        end

        function obj = estimate_dist(obj, state)
            obj.integ_dist();
        end
    end
end