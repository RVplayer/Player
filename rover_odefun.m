function [dx, out] = rover_odefun(x, u, m, a, b, Cx, Cy, CA)
    
%     m = p(1);
%     a = p(2);
%     b = p(3);
%     Cx = p(4);
%     Cy = p(5);
%     CA = p(6);
    
%     %[x y yaw vx vy r]
%     if abs(x(4)) < 0.1
%         x(4) = sign(x(4)) * 0.1;
%     end


    dx = zeros(6,1);
    dx(1) = x(4)*cos(x(3)) - x(5)*sin(x(3));
    dx(2) =  x(4)*sin(x(3)) + x(5)*cos(x(3));
    dx(3) = x(6);
    dx(4) = (x(5)*x(6) + 1/m*( Cx*(u(1)+u(2))*cos(u(5)) ...
                             -2*Cy*(u(5)-atan((x(5)+a*x(6))/x(4)))*sin(u(5)) ...
                             +Cx*(u(3)+u(4)) ...
                             -CA*x(4)^2));
    term1 = -x(4)*x(6);
    term2 = Cx*(u(1)+u(2))*sin(u(5));
    term3 = 2*Cy*(u(5)-atan((x(5)+a*x(6))/x(4)))*cos(u(5));
    term4 = 2*Cy*atan((b*x(6)-x(5))/x(4));
    out = [term1, term2, term3, term4];
    dx(5) = term1 + 1/m*(term2 + term3 + term4);
    
%     dx(5) = -x(4)*x(6) + 1/m*( Cx*(u(1)+u(2))*sin(u(5)) ...
%                               +2*Cy*(u(5)-(x(5)+a*x(6))/x(4))*cos(u(5)) ...
%                               +2*Cy*(b*x(6)-x(5))/x(4));
    dx(6) = 1/(((a+b)/2)^2*m)*( a*(Cx*(u(1)+u(2))*sin(u(5)) ...
                                  +2*Cy*(u(5)-atan((x(5)+a*x(6))/x(4)))*cos(u(5))) ...
                               -2*b*Cy*atan((b*x(6)-x(5))/x(4)));
%     if dx(5) > 10
%         disp(dx(5));
%     end
%     if abs(dx(5)) > 1.5
%         dx(5) = sign(dx(5)) * 1.5;
%     end
%     if abs(dx(6)) > 2
%         dx(6) = sign(dx(6)) * 2;
%     end
    
%     dx(1) = x(2)*x(3) + 1/m*( Cx*(u(1)+u(2))*cos(u(5)) ...
%                              -2*Cy*(u(5)-(x(2)+a*x(3))/x(1))*sin(u(5)) ...
%                              +Cx*(u(3)+u(4)) ... 
%                              -CA*x(1)^2);
%     dx(2) = -x(1)*x(3) + 1/m*( Cx*(u(1)+u(2))*sin(u(5)) ...
%                               +2*Cy*(u(5)-(x(2)+a*x(3))/x(1))*cos(u(5)) ...
%                               +2*Cy*(b*x(3)-x(2))/x(1));
%     dx(3) = 1/(((a+b)/2)^2*m) * ( a*(Cx*(u(1)+u(2))*sin(u(5)) ...
%                                     +2*Cy*(u(5)-(x(2)+a*x(3))/x(1))*cos(u(5))) ...
%                                  -2*b*Cy*(b*x(3)-x(2))/x(1));
                   
end


 