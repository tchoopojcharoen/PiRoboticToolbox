syms g real;
syms m1 m2 real;
syms l1 l2 real;
syms lc1 lc2 real;
syms cm1_y cm2_y real;
syms cm1_z cm2_z real;
syms I1_xx I1_xy I1_xz I1_yy I1_yz I1_zz real;
syms I2_xx I2_xy I2_xz I2_yy I2_yz I2_zz real;

syms q1 q2 real;
syms qd1 qd2 real;
syms qdd1 qdd2 real;
rho = [1 1]; %types of joints: 1 for revolute, 0 for prismatic 

DH_table = [q1 0 l1 0;...
            q2 0 l2 0];

gravity_vector = [0 g 0]'; % in y-axis
m = [m1 m2];
cm1 = [lc1-l1 0 0]';
cm2 = [lc2-l2 0 0]';

cm = [cm1 cm2]; %respect to generalized (local) coordinate
%frame

I1 = [0 0 0;...
      0 0 0;...
      0 0 I1_zz];

I2 = [0 0 0;...
      0 0 0;...
      0 0 I2_zz];
  
I = sym(zeros(3,3,2));
I(:,:,1) = I1;
I(:,:,2) = I2;

               