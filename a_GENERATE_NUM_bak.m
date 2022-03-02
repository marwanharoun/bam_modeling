
%% Numerical Eval
clc
hold off
% -----------------------------------------------------------------------
%                           PARAMETER VALUES
% -----------------------------------------------------------------------
TETA1a_0 = 0;
PHI1a = pi/12;
PHI3a = pi/6;
l1a = 2;
l2a = 6;
l3a = 2;
l4a = 4;
d5a_0 = 5;

% -----------------------------------------------------------------------
%                         JOINT VARIABLES VALUES
% -----------------------------------------------------------------------
TETA1a = 0;
TETA2a = 0;
TETA3a = 0;
d5a = 5;

% -----------------------------------------------------------------------
%                        POINT COORDINATES VALUES
% -----------------------------------------------------------------------
Osa_num = Osa;
O1a_num = eval(O1a);
O2a_num = eval(O2a);
Oea_num = eval(Oea);
P1a_num = eval(P1a);
P2a_num = eval(P2a);
P3a_num = eval(P3a);

% -----------------------------------------------------------------------
%                       FRAMES OF REFERENCE VALUES
% -----------------------------------------------------------------------
Ref_s_num = Ref_s;

T_1a_sa_num = eval(T_1a_sa);
Ref_1a_num = T_1a_sa_num(1:3,1:3);

T_2a_sa_num = eval(T_2a_sa);
Ref_2a_num = T_2a_sa_num(1:3,1:3);

T_ea_sa_num = eval(T_ea_sa);
Ref_ea_num = T_ea_sa_num(1:3,1:3);



% -----------------------------------------------------------------------
%      (b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)(b)
% -----------------------------------------------------------------------

T1 = T_Rot(Oea_num,T_ea_sa_num(1:3,2),-PHI3a);
T_ea_sa_num_tild = T1*T_ea_sa_num;

T2 = T_Rot(Oea_num,T_ea_sa_num_tild(1:3,1),2/3*pi);
T_eb_sa_num_tild = T2*T_ea_sa_num_tild;

T3 = T_Rot(Oea_num,T_eb_sa_num_tild(1:3,2),PHI3a);


O2b_num = T3*T2*T1*[O2a_num;1];
O2b_num = O2b_num(1:3);

P3b_num = T3*T2*T1*[P3a_num;1];
P3b_num = P3b_num(1:3);

O1b_num = T3*T2*T1*[O1a_num;1];
O1b_num = O1b_num(1:3);


P2b_num = T3*T2*T1*[P2a_num;1];
P2b_num = P2b_num(1:3);

P1b_num = T3*T2*T1*[P1a_num;1];
P1b_num = P1b_num(1:3);

Osb_num = T3*T2*T1*[Osa_num;1];
Osb_num = Osb_num(1:3);
% -----------------------------------------------------------------------
%      (c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)
% -----------------------------------------------------------------------

T4 = T_Rot(Oea_num,T_ea_sa_num(1:3,2),-PHI3a);
T_ea_sa_num_tild = T4*T_ea_sa_num;

T5 = T_Rot(Oea_num,T_ea_sa_num_tild(1:3,1),4/3*pi);
T_ec_sa_num_tild = T5*T_ea_sa_num_tild;

T6 = T_Rot(Oea_num,T_ec_sa_num_tild(1:3,2),PHI3a);

O2c_num = T6*T5*T4*[O2a_num;1];
O2c_num = O2c_num(1:3);

P3c_num = T6*T5*T4*[P3a_num;1];
P3c_num = P3c_num(1:3);

O1c_num = T6*T5*T4*[O1a_num;1];
O1c_num = O1c_num(1:3);


P2c_num = T6*T5*T4*[P2a_num;1];
P2c_num = P2c_num(1:3);

P1c_num = T6*T5*T4*[P1a_num;1];
P1c_num = P1c_num(1:3);

Osc_num = T6*T5*T4*[Osa_num;1];
Osc_num = Osc_num(1:3);

% -----------------------------------------------------------------------
%                           DRAWING THE POINTS
% -----------------------------------------------------------------------

% ----------- CUP:
Oa_num = [Osa_num';P1a_num';P2a_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)
hold on

% ----------- CUP AXIS:
Oa_num = [P1a_num';O1a_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)

% ----------- ARM:
Oa_num = [O1a_num';P3a_num';O2a_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-go','LineWidth',1)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% ----------- (P) JOINT:
Oa_num = [O2a_num';Oea_num';O2b_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-bo','LineWidth',1)

Oa_num = [O2a_num';Oea_num';O2c_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-bo','LineWidth',1)

% ----------- ARM (b):
Oa_num = [O1b_num';P3b_num';O2b_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-go','LineWidth',1)

% ----------- ARM (c):
Oa_num = [O1c_num';P3c_num';O2c_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-go','LineWidth',1)

% ----------- CUP AXIS (b):
Oa_num = [P1b_num';O1b_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)

% ----------- CUP (b):
Oa_num = [Osb_num';P1b_num';P2b_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)

% ----------- CUP AXIS (b):
Oa_num = [P1c_num';O1c_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)

% ----------- CUP (b):
Oa_num = [Osc_num';P1c_num';P2c_num'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-ro','LineWidth',1)
% -----------------------------------------------------------------------
%                     DRAWING THE FRAMES OF REFERENCE
% -----------------------------------------------------------------------

% ----------- FRAME {s}:
Oa_num = [Osa_num';Osa_num'+Ref_s_num(:,1)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [Osa_num';Osa_num'+Ref_s_num(:,2)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [Osa_num';Osa_num'+Ref_s_num(:,3)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)

% ----------- FRAME {1}:
Oa_num = [O1a_num';O1a_num'+Ref_1a_num(:,1)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [O1a_num';O1a_num'+Ref_1a_num(:,2)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [O1a_num';O1a_num'+Ref_1a_num(:,3)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)

% ----------- FRAME {2}:
Oa_num = [O2a_num';O2a_num'+Ref_2a_num(:,1)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [O2a_num';O2a_num'+Ref_2a_num(:,2)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [O2a_num';O2a_num'+Ref_2a_num(:,3)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)

% ----------- FRAME {e}:
Oa_num = [Oea_num';Oea_num'+Ref_ea_num(:,1)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [Oea_num';Oea_num'+Ref_ea_num(:,2)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)
Oa_num = [Oea_num';Oea_num'+Ref_ea_num(:,3)'];
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-kv','LineWidth',2)

grid on
axis equal

% clear *_num
% axis([-10 10 -10 10 -10 10])