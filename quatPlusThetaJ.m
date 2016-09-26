
function dq = quatPlusThetaJ(dtheta)
theta=norm(dtheta) * 0.5;
if theta < 0.244
    dq = [0.5 * dtheta;1];
else
    dq = [  0.5*dtheta(1)*sin(theta)/theta;
            0.5*dtheta(2)*sin(theta)/theta;
            0.5*dtheta(3)*sin(theta)/theta;
         cos(theta)];
end
dq = dq/norm(dq);
end

