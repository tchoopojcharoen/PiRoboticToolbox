g = 9.81;
m1 = 6.5225;
m2 = 2.0458;
l1 = 0.26;
l2 = 0.26;
lc1 = -0.0983;
lc2 = -0.0229;

cm1_x = lc1-l1;
cm2_x = lc2-l2;
cm1_y = 0;
cm2_y = 0;
cm1_z = 0;
cm2_z = 0;

I1_xx = 0;
I1_xy = 0;
I1_xz = 0;
I1_yy = 0;
I1_yz = 0;
I1_zz = 0.1213;

I2_xx = 0;
I2_xy = 0;
I2_xz = 0;
I2_yy = 0;
I2_yz = 0;
I2_zz = 0.0116;


syms q1 q2 real;
syms qd1 qd2 real;
syms qdd1 qdd2 real;
rho = [1 1]; %types of joints: 1 for revolute, 0 for prismatic 

DH_table = [q1 0 l1 0;...
            q2 0 l2 0];

gravity_vector = [0 g 0]'; % in y-axis
m = [m1 m2];
cm1 = [cm1_x cm1_y cm1_z]';
cm2 = [cm2_x cm2_y cm2_z]';

cm = [cm1 cm2]; %respect to generalized (local) coordinate
%frame

I1 = [I1_xx I1_xy I1_xz;...
      I1_xy I1_yy I1_yz;...
      I1_xz I1_yz I1_zz];

I2 = [I2_xx I2_xy I2_xz;...
      I2_xy I2_yy I2_yz;...
      I2_xz I2_yz I2_zz];
  
I = zeros(3,3,2);
I(:,:,1) = I1;
I(:,:,2) = I2;

% file:///C:/Users/tchoopojcharoen/Downloads/9781852339944-c2.pdf


               