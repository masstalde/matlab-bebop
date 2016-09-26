function R=RotFromQuatJ(q)
% if ~all(size(q) == [4, 1])
%     error('q does not have the size of a quaternion')
% end
% if abs(norm(q) - 1) > 1e-3
%     error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1')
% end


R=[q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4));
    2*(q(1)*q(2)-q(3)*q(4)),-q(1)^2+q(2)^2-q(3)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));
    2*(q(1)*q(3)+q(2)*q(4)), 2*(q(2)*q(3)-q(1)*q(4)),-q(1)^2-q(2)^2+q(3)^2+q(4)^2];
end
