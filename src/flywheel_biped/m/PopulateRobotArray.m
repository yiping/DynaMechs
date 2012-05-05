I_0 = zeros(6);

for i=1:N_ext
        j  = robot(i);
        pred = robot(i).parent;
        
        % Setup Coordinates
        robot(i).i_XL_pi = [j.pi_R_i' zeros(3); -j.pi_R_i'*cross(j.pi_p_i) j.pi_R_i'];
        
        if (i == 1) && (robot(i).fb) % if link 1 is floating base
            pi_p_i  = robot(i).q(4:6)';
            R = expm(cross(q(1:3))');
            robot(i).X_J = [R zeros(3); -R*cross(pi_p_i) R];
        
        elseif i>N
           R = eye(3);
           pi_p_i = [0 0 0]';
           robot(i).X_J = eye(6);

        else
           qi = robot(i).q;
           pi_p_i = [0 0 0]';
           R = expm(-cross(robot(i).phi(1:3)*qi));
           robot(i).X_J = [R zeros(3); zeros(3) R];
        end
        

        robot(i).i_X_pi = robot(i).X_J * robot(i).i_XL_pi;
        robot(i).pi_T_i =  [ j.pi_R_i j.pi_p_i ; 0 0 0 1]*[ R' pi_p_i ; 0 0 0 1];
        
        % Initialize inertia
        I_com_bar = j.I_com+ j.m * cross(j.i_p_com) * cross(j.i_p_com)';
        robot(i).I = [I_com_bar j.m*cross(j.i_p_com) ; j.m * cross(j.i_p_com)' j.m * eye(3)];
        
        
        % Initialize global transforms
        if j.parent == 0
            robot(i).T0 = robot(i).pi_T_i; % a.k.a. 0_T_i
            robot(i).R0 = robot(i).pi_T_i(1:3,1:3);
            robot(i).i_X_b = robot(i).i_X_pi; % maybe X0 ? b= base
        else
            robot(i).T0 = robot(pred).T0 * robot(i).pi_T_i;
            robot(i).R0 = robot(pred).R0 * robot(i).pi_T_i(1:3,1:3);
            robot(i).i_X_b = robot(i).i_X_pi * robot(pred).i_X_b;
        end
        
         if i <= N
            I_0 = I_0 + robot(i).i_X_b' * robot(i).I*robot(i).i_X_b;
         end
        
        robot(i).R0_6 = [robot(i).R0 zeros(3) ; zeros(3) robot(i).R0];
end

M = I_0(6,6);
p_com = crossExtract(I_0(1:3,4:6))/M;