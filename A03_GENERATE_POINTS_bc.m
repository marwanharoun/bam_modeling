

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
%      (c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)(c)
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



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
% ----------- (P) JOINT:
Oa_num = [Oea_num';O2b_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-bo','LineWidth',1)

Oa_num = [Oea_num';O2c_num']; 
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