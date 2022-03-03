clc
clear
syms l1a l2a l3a l4a d5a_0 d5a TETA1a_0 TETA1a TETA2a TETA3a PHI1a PHI3a



% PHI1a is the angle between the diameter of the cup alligned with the point
%   of contact, and the diameter of the cup coplanar with the red arm (see
%   figure in notes) at rest.
% PHI2a is the angle between the cup and the line passing between Os and O1
%   l12 is the distance between Os and O1.
% TETA1a_0 and d5a_0 are the starting values of joints 1 and 

% Nomenclature example: 
% Ref_1a is frame {1} of leg (a) expressed in the {s} frame.
% Ref_2a_1a is frame {2} of leg (a) (also written {2a}) expressed in
%   frame {1} of leg (a) (also written {1a}).
% O_2 is the coordinates of O_2 expressed in the {s} frame.
% O_2_1 is the coordinates of O_2 expressed in the {1} frame.
% Points and vectors are expressed in normal, not augmented, coordinates

% -----------------------------------------------------------------------
%                         FIXED GEOMETRIES
% -----------------------------------------------------------------------
PHI2a = atan2(l2a,l1a);
l12a = sqrt(l2a*l2a + l1a*l1a);

PHI5a = atan2(l2a+l3a,l1a);
l123a = sqrt(l1a*l1a + (l2a+l3a)^2);

% The following are found using the cos law:
[O1O2, PHI4a, PHI6a] = cos_law(l3a,l4a,(PHI3a+pi/2));

% -----------------------------------------------------------------------
%                         FRAMES OF (a) AT REST
% -----------------------------------------------------------------------
% ----------- Frame {s} at rest position:
Ref_s = [[1;0;0] [0;1;0] [0;0;1]];
Osa_0 = [0;0;0];
T_sa_0 = [[Ref_s; 0 0 0] [Osa_0;1]];

% ----------- Frame {1} at rest position:
Ref_1a = simplify(Rot(Ref_s(:,3), TETA1a_0)*Ref_s);
O1a_0 = [l12a*cos(PHI2a+TETA1a_0);l12a*sin(PHI2a+TETA1a_0);0];
T_1a_sa_0 = [[Ref_1a; 0 0 0] [O1a_0;1]];


% ----------- Frame {2} at rest position expressed in coordinates of {1}:
Ref_2a_1a = simplify(Rot(Ref_s(:,2), PHI1a)*Ref_s);
Ref_2a_1a = simplify(Rot(Ref_2a_1a(:,3), (PHI3a+PHI6a))*Ref_2a_1a);
Ref_2a_1a = [Ref_2a_1a(:,2) Ref_2a_1a(:,3) Ref_2a_1a(:,1)];
O2a_1a_0 = [l4a*cos(PHI3a)*cos(PHI1a);l4a*sin(PHI3a)+l3a;l4a*cos(PHI3a)*-sin(PHI1a)]; 
% (It's different from the one found in my notes. Verify)
T_2a_1a_0 = [[Ref_2a_1a; 0 0 0] [O2a_1a_0;1]];

% ----------- Frame {2} at rest position:
Ref_2a = simplify(Rot(Ref_1a(:,2), PHI1a)*Ref_1a);
Ref_2a = simplify(Rot(Ref_2a(:,3), (PHI3a+PHI6a))*Ref_2a);
Ref_2a = [Ref_2a(:,2) Ref_2a(:,3) Ref_2a(:,1)];
O2a_0 = T_1a_sa_0*[O2a_1a_0;1];
O2a_0 = O2a_0(1:3);
T_2a_sa_0 = [[Ref_2a;0 0 0] [O2a_0;1]];

% ----------- Frame {e} at rest position:
Ref_ea = Ref_2a;
Oea_2a_0 = [0;0;d5a_0];
Oea_0 = T_1a_sa_0*T_2a_1a_0*[Oea_2a_0;1];
Oea_0 = Oea_0(1:3);
T_ea_sa_0 = [[Ref_ea;0 0 0] [Oea_0;1]];


% -----------------------------------------------------------------------
%                     OTHER POINTS OF (a) AT REST
% -----------------------------------------------------------------------
P1a_0 = [l1a*cos(TETA1a_0);l1a*sin(TETA1a_0);0];
P2a_0 = [2*l1a*cos(TETA1a_0);2*l1a*sin(TETA1a_0);0];
P3a_0 = [l123a*cos(PHI5a+TETA1a_0);l123a*sin(PHI5a+TETA1a_0);0];

% -----------------------------------------------------------------------
%                         AXES OF (a) AT REST
% -----------------------------------------------------------------------
Zsa = Ref_s(:,3);
Z1a = Ref_1a(:,3);
X1a = Ref_1a(:,1);
Z2a = Ref_2a(:,3);

% -----------------------------------------------------------------------
%           DISPLACEMENT MATRICES (DEPENDANT ON q) OF (a) (IN {s})
% -----------------------------------------------------------------------
T1a = T_Rot(Osa_0,Zsa,TETA1a);
T2a = T_Rot(O1a_0,Z1a,TETA2a);
T3a = T_Rot(O1a_0,X1a,TETA3a);
T4a = T_Trans(Z2a,d5a);

% -----------------------------------------------------------------------
%                         FRAMES OF (a) NOT AT REST
% -----------------------------------------------------------------------
T_sa = T_sa_0;
T_1a_sa = T1a*T_1a_sa_0;
T_2a_sa = T1a*T2a*T3a*T_2a_sa_0;
T_ea_sa = T1a*T2a*T3a*T4a*T_ea_sa_0;

% -----------------------------------------------------------------------
%                         ORIGINS OF (a) NOT AT REST
% -----------------------------------------------------------------------
Osa = Osa_0;
O1a = T_1a_sa(1:3,4);
O2a = T_2a_sa(1:3,4);
Oea = T_ea_sa(1:3,4);

% -----------------------------------------------------------------------
%                         POINTS OF (a) NOT AT REST
% -----------------------------------------------------------------------
P1a = T1a*[P1a_0;1];
P1a = P1a(1:3);
P2a = T1a*[P2a_0;1];
P2a = P2a(1:3);
P3a = T1a*T2a*T3a*[P3a_0;1];
P3a = P3a(1:3);




