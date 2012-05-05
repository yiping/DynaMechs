N=3;
N_joint = 3;
N_ext = 4; % extended links

%
robot(1).parent = 0;
robot(1).phi    = [0 0 1 0 0 0]';
robot(1).i_p_com  = [0 0 0]';
robot(1).m      = 0.00001;
robot(1).I_com  = diag([1e-8 1e-8 1e-8])  ;
robot(1).pi_p_i   = [0,0,0.46]';
robot(1).pi_R_i   = [0 -1  0
                     1  0  0
                     0  0  1];
robot(1).joints = [1];
robot(1).q      =  q(1);
robot(1).fb = 0; % This is not a floating base

%
robot(2).parent = 1;
robot(2).phi    = [0 0 1 0 0 0]';
robot(2).i_p_com  = [0 -2.08 0]';
robot(2).m      = 0.00000027;
robot(2).I_com  =  1.0e-06*diag([0.1039  0.41 0.1039]);
robot(2).pi_p_i   = [0 0 0]';
robot(2).pi_R_i   = [0 -1  0
                     0  0 -1
                     1  0  0];
robot(2).joints  = [2];
robot(2).q      = q(2);

%
robot(3).parent = 2;
robot(3).phi    = [0 0 1 0 0 0]';
robot(3).i_p_com  = [0.038	0.0008	0]';
robot(3).m      = 12.1;
robot(3).I_com  =   [  0.2180    0.0031    0.0117
                       0.0031    0.2395    0.0048
                       0.0117    0.0048    0.1035];
robot(3).pi_p_i   = [0 -2.08 0]';
robot(3).pi_R_i   = [1  0  0
                     0  0 -1
                     0  1  0];
robot(3).joints  = [3];
robot(3).q      = q(3);

% extended link, think as end effector
robot(4).parent = 3;
robot(4).phi    = []';
robot(4).i_p_com  = [0 0 0]';
robot(4).m      = 0;
robot(4).I_com  = diag([0,0,0]);
robot(4).pi_p_i   = [0.2,0,0]';
robot(4).pi_R_i   = eye(3);
robot(4).joints = [];
robot(4).q      = [];
