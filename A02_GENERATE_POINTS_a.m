
%% Numerical Eval
clc
hold off
% -----------------------------------------------------------------------
%                           PARAMETER VALUES
% -----------------------------------------------------------------------
TETA1a_0 = pi/12;
PHI1a = pi/12;
PHI3a = pi/6;
l1a = 2;
l2a = 6;
l3a = 5; % if l3a = 0, the arm will pass through O1a all the way.
l4a = 5;
d5a_0 = 5;

% -----------------------------------------------------------------------
%                         JOINT VARIABLES VALUES
% -----------------------------------------------------------------------
TETA1a = 0;
TETA2a = 0;
TETA3a = pi/12;
TETA4a = pi/4;
d5a = 0;

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

% ----------- (P) JOINT:
Oa_num = [O2a_num';Oea_num']; 
plot3(Oa_num(:,3),Oa_num(:,1),Oa_num(:,2),'-bo','LineWidth',1)

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