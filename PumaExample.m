%PUMA TARN from http://academic.csuohio.edu/richter_h/courses/mce647/Corke-Armstrong.pdf
%Ari Goodman

DH = [0  0       0      -pi/2;
      0  0       431.8   0; 
      0  150.05 -19.1    pi/2;
      0  431.1   0       -pi/2; 
      0  0       0       pi/2;
      0  0       0       0]; 

mass = [13.00 22.40 5.00 1.20 0.62 0.16]; %1xn
centerOfMass = [0 103 20 0 0 0; 4 5 -4 -3 -1 0; -309 -40 14 -86 -10 3]; %3xn
inertia = [1.100, 0, 0; 0 1.11 0; 0 0 .177;];%3x3xn
inertia(:,:,2) = [0.403, 0, 0; 0 0.969 0; 0 0 .965;];
inertia(:,:,3) = [74.8E-3, 0, 0; 0 7.3E-3 0; 0 0 324E-3;];
inertia(:,:,4) = [5.32E-3, 0, 0; 0 5.2E-3 0; 0 0 2.5E-3;];
inertia(:,:,5) = [487E-6, 0, 0; 0 482E-6 0; 0 0 348E-6;];
inertia(:,:,6) = [123E-6, 0, 0; 0 123E-6 0; 0 0 13E-6;];

rho = [1 1 1 1 0 1];
gravity_vector = [0; -9.81; 0];
isSym = 1; %ERROR IF 0

Puma = Manipulator(DH,mass,centerOfMass,inertia,rho,gravity_vector,isSym)

%{
    OUTPUT

Puma = 

  Manipulator with properties:

          Environment: [1x1 struct]
         DHParameters: [6x4 double]
    DynamicParameters: [1x1 struct]
             LinkList: [1x1 LinkTree]
                State: [1x1 struct]

%}
