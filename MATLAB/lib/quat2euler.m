function [rol, pit, yaw] = quat2euler(quaternion)
    % change quaternion to euler roll, pitch, yaw angle
    e0 = quaternion(1);
    ex = quaternion(2);
    ey = quaternion(3);
    ez = quaternion(4);
    rol = atan2(2*(e0*ex + ey*ez), (e0^2 + ez^2 - ex^2 - ey^2));
    pit = asin(2*(e0*ey - ex*ez));
    yaw = atan2(2*(e0*ez + ex*ey), (e0^2 + ex^2 - ey^2 - ez^2));
end