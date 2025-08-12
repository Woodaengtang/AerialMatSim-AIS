classdef WindDryden < handle
    
    properties
        T  % Sampling time
        
        V_ws % Ambient wind vector R3

        Lu % Spatial wave lengths for u component
        Lv % Spatial wave lengths for v component
        Lw % Spatial wave lengths for w component

        sig_u   % Intensities of turbulence for u component
        sig_v   % Intensities of turbulence for v component
        sig_w   % Intensities of turbulence for w component

        Hu     % Filter output for u component
        Hu_prev % Previous filter output for u component
        Hv     % Filter output for v component
        Hv_prev % Previous filter output for v component
        Hv_pprev % Pre-previous filter output for v component
        Hw     % Filter output for w component
        Hw_prev % Previous filter output for w component
        Hw_pprev % Pre-previous filter output for w component

        Wu     % White noise for u component
        Wu_prev % Previous white noise for u component
        Wv     % White noise for v component
        Wv_prev % Previous white noise for v component
        Ww     % White noise for w component
        Ww_prev % Previous white noise for w component

        model_params
    end

    methods
        function obj = WindDryden(sampling_time, ambient_wind)
            obj.T = sampling_time;
            
            obj.V_ws = ambient_wind;

            obj.Lu = 0;
            obj.Lv = 0;
            obj.Lw = 0;

            obj.sig_u = 0;
            obj.sig_v = 0;
            obj.sig_w = 0;

            obj.Hu = 0;
            obj.Hu_prev = 0;
            obj.Hv = 0;
            obj.Hv_prev = 0;
            obj.Hv_pprev = 0;
            obj.Hw = 0;
            obj.Hw_prev = 0;
            obj.Hw_pprev = 0;

            obj.Wu = 0;
            obj.Wu_prev = 0;
            obj.Wv = 0;
            obj.Wv_prev = 0;
            obj.Ww = 0;
            obj.Ww_prev = 0;

            obj.model_params = struct('alt_0', 50,...
                                      'alt_e', 600,...
                                      'L_u_0', 200,...
                                      'L_u_e', 533,...
                                      'L_v_0', 200,...
                                      'L_v_e', 533,...
                                      'L_w_0', 50,...
                                      'L_w_e', 533,...
                                      'sig_u_l_0', 1.06,...
                                      'sig_u_l_e', 1.5,...
                                      'sig_v_l_0', 1.06,...
                                      'sig_v_l_e', 1.5,...
                                      'sig_w_l_0', 0.7,...
                                      'sig_w_l_e', 1.5,...
                                      'sig_u_m_0', 2.12,...
                                      'sig_u_m_e', 3.0,...
                                      'sig_v_m_0', 2.12,...
                                      'sig_v_m_e', 3.0,...
                                      'sig_w_m_0', 1.4,...
                                      'sig_w_m_e', 3.0);
        end

        function V_w = update_wind(obj, height, Va, euler)
            if isstruct(euler)
                rol = euler.rol;
                pit = euler.pit;
                yaw = euler.yaw;
            elseif isvector(euler)
                rol = euler(1);
                pit = euler(2);
                yaw = euler(3);
            else    
                error('Input variable euler must be a struct.');
            end
            rotm_eul = eul2rotm([-rol, -pit, -yaw], 'ZYX');
            Vb_ws = rotm_eul*obj.V_ws;
            obj.update_gust(height, Va);
            V_w = [obj.Hu; obj.Hv; obj.Hw] + Vb_ws;
        end
        
        function obj = update_gust(obj, height, Va)
            obj.set_wave_length(height);
            obj.set_turb_intense(height);
            
            obj.Wu = randn(1);
            obj.Wv = randn(1);
            obj.Ww = randn(1);
            
            obj.update_Hu(Va);
            obj.update_Hv(Va);
            obj.update_Hw(Va);
        end
        
        function obj = set_wave_length(obj, height)
            % This function determines the wave length parameters based on the height of the aircraft
            obj.Lu = (obj.model_params.L_u_e - obj.model_params.L_u_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.L_u_0;
            obj.Lv = (obj.model_params.L_v_e - obj.model_params.L_v_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.L_v_0;
            obj.Lw = (obj.model_params.L_w_e - obj.model_params.L_w_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.L_w_0;
        end

        function obj = set_turb_intense(obj, height)
            % This function determines the turbulence intensity based on the height of the aircraft
            turb_intense = 1; % 1 indicates moderate turbulence, 0 indicates light turbulence
            if turb_intense
                % Moderate turbulence
                obj.sig_u = (obj.model_params.sig_u_m_e - obj.model_params.sig_u_m_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_u_m_0;
                obj.sig_v = (obj.model_params.sig_v_m_e - obj.model_params.sig_v_m_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_v_m_0;
                obj.sig_w = (obj.model_params.sig_w_m_e - obj.model_params.sig_w_m_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_w_m_0;
            else
                % Light turbulence
                obj.sig_u = (obj.model_params.sig_u_l_e - obj.model_params.sig_u_l_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_u_l_0;
                obj.sig_v = (obj.model_params.sig_v_l_e - obj.model_params.sig_v_l_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_v_l_0;
                obj.sig_w = (obj.model_params.sig_w_l_e - obj.model_params.sig_w_l_0)/(obj.model_params.alt_e - obj.model_params.alt_0)*(height - obj.model_params.alt_0) + obj.model_params.sig_w_l_0;
            end
        end

        function obj = update_Hu(obj, Va)
            VaLu = Va/obj.Lu;
            alpha = exp(-VaLu*obj.T);
            obj.Hu = alpha*obj.Hu_prev + obj.sig_u*sqrt(2*VaLu/pi)*(1 - alpha)*obj.Wu;
            obj.Hu_prev = obj.Hu;
        end

        function obj = update_Hv(obj, Va)
            a = Va/obj.Lv;
            b = a/sqrt(3);
            alpha = exp(-a*obj.T);

            obj.Hv = 2*alpha*obj.Hv_prev - alpha^2*obj.Hv_pprev + obj.sig_v*sqrt(3*a/pi)*((obj.T*alpha + b/a^2*(1 - alpha - a*obj.T*alpha))*obj.Wv + (-obj.T*alpha + b/a^2*(alpha^2 - alpha + a*obj.T*alpha))*obj.Wv_prev);
            obj.Hv_pprev = obj.Hv_prev;
            obj.Hv_prev = obj.Hv;
            obj.Wv_prev = obj.Wv;
        end

        function obj = update_Hw(obj, Va)
            a = Va/obj.Lw;
            b = a/sqrt(3);
            alpha = exp(-a*obj.T);

            obj.Hw = 2*alpha*obj.Hw_prev - alpha^2*obj.Hw_pprev + obj.sig_w*sqrt(3*a/pi)*((obj.T*alpha + b/a^2*(1 - alpha - a*obj.T*alpha))*obj.Ww + (-obj.T*alpha + b/a^2*(alpha^2 - alpha + a*obj.T*alpha))*obj.Ww_prev);
            obj.Hw_pprev = obj.Hw_prev;
            obj.Hw_prev = obj.Hw;
            obj.Ww_prev = obj.Ww;
        end
    end
end