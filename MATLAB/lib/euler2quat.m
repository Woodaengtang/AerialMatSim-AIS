function [e0, ex, ey, ez] = euler2quat(euler)
    cosrol = cos(euler(1)/2);
    sinrol = sin(euler(1)/2);
    cospit = cos(euler(2)/2);
    sinpit = sin(euler(2)/2);
    cosyaw = cos(euler(3)/2);
    sinyaw = sin(euler(3)/2);
    e0 = cosyaw*cospit*cosrol + sinyaw*sinpit*sinrol;
    ex = cosyaw*cospit*sinrol - sinyaw*sinpit*cosrol;
    ey = cosyaw*sinpit*cosrol + sinyaw*cospit*sinrol;
    ez = sinyaw*cospit*cosrol - cosyaw*sinpit*sinrol;
end