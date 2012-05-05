for i=1:N
    joints = robot(i).joints;
    robot(i).q = q(joints);
    robot(i).qd = qd(joints);
end