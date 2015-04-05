%PUMA 560 in mm
%Ari Goodman
%UNFINISHED

DH = [pi/2 -pi/2 0 0; %-160 160
        0 0 431.8 149.09; %-225 45
        pi/2 pi/2 -20.32 0; %-45 225
        0 -pi/2 0 433.07; %-110 170
        0 pi/2 0 0; %-100 100
        0 0 0 56.25]; %-266 266

mass = [1 1 1 1 1 1]; %1xn
centerOfMass = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]; %3xn
inertia = [1, 1, 1; 1 1 1; 1 1 1;];%3x3xn
inertia(:,:,2) = [1, 1, 1; 1 1 1; 1 1 1;];
inertia(:,:,3) = [1, 1, 1; 1 1 1; 1 1 1;];
inertia(:,:,4) = [1, 1, 1; 1 1 1; 1 1 1;];
inertia(:,:,5) = [1, 1, 1; 1 1 1; 1 1 1;];
inertia(:,:,6) = [1, 1, 1; 1 1 1; 1 1 1;];

rho = [1 1 1 1 1 1];
gravity_vector =0;
isSym = 1; %ERROR IF 0

Puma = Manipulator(DH,mass,centerOfMass,inertia,rho,gravity_vector,isSym)
