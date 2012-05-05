
q=[0 0 0.5  ]';
qd = [0 0 0 ]';

defineBiped

SetState;    
PopulateRobotArray;
% ComputeH
% ComputeCandG

p_com0 = p_com;
drawBiped


dt = .001;
T = 0:dt:4;
Q = zeros( length(T), 3);
Q(:,1) = 0.05*(1-cos(pi/4*T));
P=[];
ROT=[];

for cnt = 1:length(T)
    t = T(cnt);

    q = Q(cnt,:)';
    SetState;
    PopulateRobotArray;
    if mod(cnt,10) == 0
        drawBiped;
        fprintf('%d \t %f \t \n',cnt,t);
        pause(.001);
    end

    robot(3).b_X_i = (robot(3).i_X_b)\eye(6); 
    robot(3).b_R_i = robot(3).b_X_i(1:3,1:3);
    robot(3).b_p_i = crossExtract(robot(3).b_X_i(4:6,1:3)*(robot(3).b_R_i\eye(3)) );
    
    P = [P; robot(3).b_p_i];
    ROT = [ROT; robot(3).b_R_i(1,:), robot(3).b_R_i(2,:), robot(3).b_R_i(3,:)];
end

P(:,1) = P(:,1)+3;
P(:,2) = P(:,2)+3;
fileID = fopen('sp.txt','w');
formatSpec = '%f\t %f\t %f \t\n';
fprintf(fileID, formatSpec, P');

fileID = fopen('sR.txt','w');
formatSpec = '%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t\n';
fprintf(fileID, formatSpec, ROT');


% dt = .001;
% T = 0:dt:.5;
% LenT = length(T);
% cntEnd = LenT;
% 
% for cnt = 1:cntEnd
%     qd2 = qd;
%     qd2(1:6) = robot(1).R0_6'*qd(1:6);
%     
%     Q(:,cnt) = q';
%     Qd(:,cnt) = qd;
%     t = T(cnt);
%     
%     SetState;
%     PopulateRobotArray;
%     PopulateRobotArray;
%     robot(1).fb = 1;t = T(cnt);
%     
%     if mod(cnt,10) == 0
%         DrawRobot;
%         fprintf('%d \t %f \t %f\n',cnt,t,max(abs(qd)));
%         pause(.001);
%     end
%     
%     ComputeH;
%     ComputeCandG;
%     ABInitialization;
% 
%     tau1 = sin(t);
%     tau2 = cos(t);
%     tau = [0 0 0 0 0 0 tau1 tau2]';
%     
%     ABBackwardsDynamics;
%     ABForwardsAccelerations;
%    
%     %Process FB qdd
%     v = robot(1).v;
%     a = qdd(1:6) + [zeros(3,1) ; cross(v(1:3))*v(4:6)];
%     qdd(1:6) = robot(1).R0_6 * a;
% 
%     q = q + qd' * dt;
%     qd = qd + qdd * dt;
% 
% 
%     quatdot = omegaToQuatRates(qd(1:3),quat);
%     quat = quat + dt * quatdot;
%     quat = quat/norm(quat);
%     q(1:3) = quatToAngAx(quat);    
% end
% com_excursion = p_com-p_com0