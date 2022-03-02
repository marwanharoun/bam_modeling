% Quelques tests pour le TP3

%% dcm2eul
R = [0.790482, 0.0117385, 0.612372;
     0.48368, -0.625343, -0.612372;
     0.375754, 0.780262,   -0.5];
[phi,theta,psi] = dcm2eul(R,0);
erreurangles1 = abs(exp(1i*(phi-pi/4))-1)+abs(exp(1i*(theta-2*pi/3))-1)+...
  abs(exp(1i*(psi-pi/7))-1)

[phi,theta,psi] = dcm2eul(R,1);
erreurangles2 = abs(exp(1i*(phi-5*pi/4))-1)+abs(exp(1i*(theta+2*pi/3))-1)+...
  abs(exp(1i*(psi-8*pi/7))-1)

%% ikine_anthroRRR
Wdes = [0,-10,768.3];  
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"ru")
Wdes = [525,-10,243.3]; 
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"ru")
Wdes = [450,-10,243.3];  
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"ru")
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"rd")
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"lu")
[theta1,theta2,theta3] = ikine_anthroRRR(Wdes,"ld")

%% ikine_anthroRRR - suite
L1=128.3+115; L2=280; L3=140+105; shoulderoffset=30;  % à remplir
thetas=rand(1,3)*2*pi-pi;  % validation sur des configurations aléatoires
T=cindir_gen3_03(thetas);  % pas demandé mais conseillé d'implémenter une telle fonction
[theta1,theta2,theta3]=ikine_anthroRRR(T(1:3,4),"ru");
[theta12,theta22,theta32]=ikine_anthroRRR(T(1:3,4),"rd");
[theta13,theta23,theta33]=ikine_anthroRRR(T(1:3,4),"lu");
[theta14,theta24,theta34]=ikine_anthroRRR(T(1:3,4),"ld");

disp("--------------------")
disp("Original variables")
disp(thetas)
disp("Sol ru:")
disp([theta1,theta2,theta3])
disp("Sol rd:")
disp([theta12,theta22,theta32])
disp("Sol lu:")
disp([theta13,theta23,theta33])
disp("Sol ld:")
disp([theta14,theta24,theta34])
disp("--------------------")

%% ikien_gen3lite_simplified
testconfig = rand(1,6)*2*pi-pi;
%testconfig = zeros(1,6);
%testconfig2 = pi/6 * ones(1,6);
%testconfig3 = rand(1,6);

% il est conseillé d'implémenter cette fonction de cin. directe (voir le TP2), par exemple avec le robotics toolbox
testpose = cindir_gen3(testconfig);  

qvars = zeros(8,6);
qvars(1,:) = ikine_gen3lite_simplifie(double(testpose), "ru", 0);
qvars(2,:) = ikine_gen3lite_simplifie(double(testpose), "rd", 0);
qvars(3,:) = ikine_gen3lite_simplifie(double(testpose), "lu", 0);
qvars(4,:) = ikine_gen3lite_simplifie(double(testpose), "ld", 0);
qvars(5,:) = ikine_gen3lite_simplifie(double(testpose), "ru", 1);
qvars(6,:) = ikine_gen3lite_simplifie(double(testpose), "rd", 1);
qvars(7,:) = ikine_gen3lite_simplifie(double(testpose), "lu", 1);
qvars(8,:) = ikine_gen3lite_simplifie(double(testpose), "ld", 1);
% On teste qu'une des 8 configurations est celle dont on est parti
myerror = 10;
for k=1:8
    myerror = min(myerror,norm(testconfig-qvars(k,:)));
end
disp(myerror)

%% ikien_gen3lite
% Une approche similaire à ci-dessus peut être suivie
% Quelques poses à tester en plus


testconfig = [-2.2210, -1.9536, -2.8736, 0.8495, -1.3706, 0.2425];
testpose = [ 0.2700 -0.3889  0.8808 180.1
            -0.9012 -0.4241  0.0890 -68.67
             0.3389 -0.8179 -0.4650 207.8
             0       0       0        1];


%{
% Est-ce que cette pair (configuration,pose est retrouvée?
testconfig = [2.6912, 0.5032, -3.0349, -2.3822, 2.2790, -0.0987];
testpose = [-0.2182 -0.8670 -0.4479 -81.77
             0.9182 -0.0268 -0.3953 -49.55
             0.3307 -0.4975  0.8019 467
             0       0       0        1];
%}

qvars = zeros(8,6);
try  % il est aussi possible que le modèle simplifié n'atteigne pas la pose, dans ce cas ou initialise différemment
    tmp = ikine_gen3lite_simplified(double(testpose), "ru", 0);
    qvars(1,:) = ikine_gen3lite(double(testpose), tmp(6), "ru", 0);
catch
    qvars(1,:) = ikine_gen3lite(double(testpose), 0, "ru", 0);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "rd", 0);
    qvars(2,:) = ikine_gen3lite(double(testpose), tmp(6), "rd", 0);
catch
    qvars(2,:) = ikine_gen3lite(double(testpose), 0, "rd", 0);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "lu", 0);
    qvars(3,:) = ikine_gen3lite(double(testpose), tmp(6), "lu", 0);
catch
    qvars(3,:) = ikine_gen3lite(double(testpose), 0, "lu", 0);
end
try 
    tmp = ikine_gen3lite_simplifie(double(testpose), "ld", 0);
    qvars(4,:) = ikine_gen3lite(double(testpose), tmp(6), "ld", 0);
catch
    qvars(4,:) = ikine_gen3lite(double(testpose), 0, "ld", 0);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "ru", 1);
    qvars(5,:) = ikine_gen3lite(double(testpose), tmp(6), "ru", 1);
catch 
    qvars(5,:) = ikine_gen3lite(double(testpose), 0, "ru", 1);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "rd", 1);
    qvars(6,:) = ikine_gen3lite(double(testpose), tmp(6), "rd", 1);
catch 
    qvars(6,:) = ikine_gen3lite(double(testpose), 0, "rd", 1);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "lu", 1);
    qvars(7,:) = ikine_gen3lite(double(testpose), tmp(6), "lu", 1);
catch 
    qvars(7,:) = ikine_gen3lite(double(testpose), 0, "lu", 1);
end
try
    tmp = ikine_gen3lite_simplifie(double(testpose), "ld", 1);
    qvars(8,:) = ikine_gen3lite(double(testpose), tmp(6), "ld", 1);
catch 
    qvars(8,:) = ikine_gen3lite(double(testpose), 0, "ld", 1);
end
testconfig
qvars
