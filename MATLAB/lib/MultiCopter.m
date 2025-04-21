classdef MultiCopter < handle

    properties
        pos
        pos_d
        pn
        pe
        pd

        vel
        vel_d
        vn
        ve
        vd

        quat
        quat_d
        e0
        ex
        ey
        ez

        att
        att_d
        rol
        pit
        yaw

        omg
        omg_d
        p
        q
        r

        T
        Mx
        My
        Mz
    end
    
    methods
        function obj = MultiCopter(initCond)
            obj.pn = initCond.p(1);
            obj.pe = initCond.p(2);
            obj.pd = initCond.p(3);
            obj.pos = [obj.pn, obj.pe, obj.pd]';

            obj.vn = initCond.v(1);
            obj.ve = initCond.v(2);
            obj.vd = initCond.v(3);
            obj.vel = [obj.vn, obj.ve, obj.vd]';

            obj.e0 = initCond.q(1);
            obj.ex = initCond.q(2);
            obj.ey = initCond.q(3);
            obj.ez = initCond.q(4);
            obj.quat = [obj.e0, obj.ex, obj.ey, obj.ez]';

            [obj.rol, obj.pit, obj.yaw] = quat2euler(initCond.q);
            obj.att = [obj.rol, obj.pit, obj.yaw]';

            obj.p = initCond.omega(1);
            obj.q = initCond.omega(2);
            obj.r = initCond.omega(3);
            obj.omg = [obj.p, obj.q, obj.r];

            obj.pos_d = NaN([3, 1]);
            obj.vel_d = NaN([3, 1]);
            obj.quat_d = NaN([4, 1]);
            obj.att_d = NaN([3, 1]);
            obj.omg_d = NaN([3, 1]);

            obj.T = NaN;
            obj.Mx = NaN;
            obj.My = NaN;
            obj.Mz = NaN;
        end
        
        function obj = update_states(obj)

        end
    end
end


