clear;
clc;


syms D1 D2 D3 TETA4 TETA5 TETA6 
syms L3 D_we

% Matrice nulle et matrice unitaire
ZERO = [   
    0 0 0;
    0 0 0;
    0 0 0    
];

I = [    
    1 0 0;
    0 1 0;
    0 0 1    
];

% Axe de chaque articulation
Z1 = [0;        0;        1];
Z2 = [0;        1;        0];
Z3 = [1;        0;        0];
Z4 = [1;        0;        0];
Z5 = [0;        1;        0];
Z6 = [1;        0;        0];

% Centre du repère de chaque articulation à la configuration de repos
O0 = [0;        0;        0];
O1 = [0;        0;        0];
O2 = [0;        0;        0];
O3 = [0;        0;        0];
O4 = [L3;       0;        0];
O5 = [L3;       0;        0];
O6 = [L3;       0;        0];
E0 = [L3+D_we;        0;        0];

% Pose de l'éffecteur en configuration de repos
Tse0 = [
    0  0  1 E0(1);
    0 -1  0 E0(2);
    1  0  0 E0(3);
    0  0  0 1
];

Rse0 = [Tse0(1,1) Tse0(1,2) Tse0(1,3);Tse0(2,1) Tse0(2,2) Tse0(2,3);Tse0(3,1) Tse0(3,2) Tse0(3,3)];


% Pose de chaque articulation
T1 = T_Trans(Z1,D1);
T2 = T_Trans(Z2,D2);
T3 = T_Trans(Z3,D3);
T4 = T_Rot(O4,Z4,TETA4);
T5 = T_Rot(O5,Z5,TETA5);
T6 = T_Rot(O6,Z6,TETA6);

Tse = simplify(T1*T2*T3*T4*T5*T6*Tse0,'steps',10); % NE PAS OUBLIER!!!!

D1 = 1.1;
D2 = 0.4;
D3 = 0.7;

TETA4 = pi/7;
TETA5 = pi/5;
TETA6 = pi/6;

L3 = 2.3;
D_we = 1.7;

Tse_question_c = eval(Tse);

% Cin inv:
T3_e = T4*T5*T6*Tse0;
R3_e = [T3_e(1,1) T3_e(1,2) T3_e(1,3);T3_e(2,1) T3_e(2,2) T3_e(2,3);T3_e(3,1) T3_e(3,2) T3_e(3,3)];

syms nx ny nz ox oy oz ax ay az
syms D1 D2 D3 TETA4 TETA5 TETA6 

Rse_d = [[nx;ny;nz] [ox;oy;oz] [ax;ay;az]];

Tse_d = [[0.572061;-0.789312;-0.223006;0] [-0.415627;-0.044565;-0.908443;0] [0.707107;0.612372;-0.353553;0] [4.40208;1.84103;0.0989592;1]]

% Tse_d = Tse_question_c; % To test in with other data

Rse_d = [Tse_d(1,1) Tse_d(1,2) Tse_d(1,3);Tse_d(2,1) Tse_d(2,2) Tse_d(2,3);Tse_d(3,1) Tse_d(3,2) Tse_d(3,3)];

TETA5_inv = acos(Rse_d(1,3));
TETA6_inv = acos(Rse_d(1,1)/sin(TETA5_inv));
TETA4_inv = acos(-Rse_d(3,3)/sin(TETA5_inv));


res_D1 = Tse_d(3,4) - D_we*Tse_d(3,3);
res_D2 = Tse_d(2,4) - D_we*Tse_d(2,3);
res_D3 = Tse_d(1,4) - D_we*Tse_d(1,3) - L3;
res_TETA4 = TETA4_inv;
res_TETA5 = TETA5_inv;
res_TETA6 = TETA6_inv;

D1 = res_D1;
D2 = res_D2;
D3 = res_D3;
TETA4 = res_TETA4;
TETA5 = res_TETA5;
TETA6 = res_TETA6;

Tse_cininv = eval(Tse);
R3_e_cininv = eval(R3_e);
test = Tse_cininv - Tse_d

% Jacobienne:

syms D1 D2 D3 TETA4 TETA5 TETA6 
syms L3 D_we

% Torseur de chaque articulation au repos
V1 = [0;0;0;Z1];
V2 = [0;0;0;Z2];
V3 = [0;0;0;Z3];
V4 = [Z4;-cross(Z4,O4)];
V5 = [Z5;-cross(Z5,O5)];
V6 = [Z6;-cross(Z6,O6)];

% Colonnes de la Jacobienne. NE PAS OUBLIER AdT !!!
J1 = V1
J2 = simplify(AdT(T1)*V2,'steps',10)
J3 = simplify(AdT(T1*T2)*V3,'steps',10)
J4 = simplify(AdT(T1*T2*T3)*V4,'steps',10);
J5 = simplify(AdT(T1*T2*T3*T4)*V5,'steps',10);
J6 = simplify(AdT(T1*T2*T3*T4*T5)*V6,'steps',10);

% La matrice suivante est la jacobienne à un point coincidant
% instatanement avec O. 
J = [J1 J2 J3 J4 J5 J6];


% Pour trouver la matrice à un point coincidant instantanement avec E,
% voir 5.28. 




% Coordonnées de E:
E = [
    Tse(1,4); 
    Tse(2,4); 
    Tse(3,4)
]; % En fait ici E est OE. 


%Ensuite on trouve la matrice antisymetrique de ça
Ex = x(E);


% Et maintenant la matrice dans 5.28   
M = [
    I,ZERO;
    -Ex,I        
];

% Ce qui nous permet de trouver la jacobienne désirée:
Je = M*J;

D1 = 1.5;
D2 = 1;
D3 = 0.5;
TETA4 = pi/3;
TETA5 = pi/4;
TETA6 = pi/5;

L3 = 2.3;
D_we = 1.7;

Je_question_h = eval(Je);

Vse_question_h = Je_question_h*[0; 0; 0; 0; 1; 0];



% Singularités:

det_J = det(J);
inv_J = inv(J);




% Lagrange:
syms m1 m2 m3 d1_dot d2_dot d3_dot g
syms D1 D2 D3

q_dot = [d1_dot; d2_dot; d3_dot]
J0 = [0;0;0;0;0;0]

J_k1 = [J1 J0 J0]
J_k2 = [J1 J2 J0]
J_k3 = [J1 J2 J3]

K1=0.5*m1*transpose(q_dot)*transpose(J_k1)*J_k1*q_dot
K2=0.5*m2*transpose(q_dot)*transpose(J_k2)*J_k2*q_dot
K3=0.5*m3*transpose(q_dot)*transpose(J_k3)*J_k3*q_dot

P1 = -0.5*m1*g*D1
P2 = -m2*g*D1
P3 = -m3*g*D1

LAGRANGE = K1+K2+K3-(P1+P2+P3)