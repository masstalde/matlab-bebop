function qp=quatmultJ(q,p)
if coder.target('MATLAB')
    if abs(norm(q) - 1) > 1e-1
        error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1 %s', mat2str(q))
    end
    if abs(norm(p) - 1) > 1e-1
        error('The provided quaternion is not a valid rotation quaternion because it does not have norm 1: %s', mat2str(p))
    end
end
p1=p(1);
p2=p(2);
p3=p(3);
p4=p(4);

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
qp=[p4,-p3,p2,p1;
    p3,p4,-p1,p2;
    -p2,p1,p4,p3;
    -p1,-p2,-p3,p4]*[q1;q2;q3;q4];
qp = qp/norm(qp);
end
