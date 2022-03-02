function [Tsw_cindir,Ts3] = cindir_gen3_03(q)

    format bank
    % Axe de chaque articulation
    Z1 = [0; 0; 1];
    Z2 = [0; -1; 0];
    Z3 = [0; 1; 0];

    % Centre du repère de chaque articulation à la configuration de repos
    O1 = [0;             0        ;        0                               ];
    O2 = [0;             0        ;        115+128.3                       ];
    O3 = [0;             -30      ;        115+128.3+280                   ];
    W0 = [0;             -30      ;        115+128.3+280+140+105           ];

    % Pose de W en configuration de repos:
    Tsw0 = [
        0 1 0 W0(1);
        0 0 1 W0(2);
        1 0 0 W0(3);
        0 0 0 1
    ];

% % % %     TO BE DELETED:::::
% % % %     % Pose de l'éffecteur en configuration de repos
% % % %     Tsw0 = [
% % % %         1 0 0 W0(1);
% % % %         0 1 0 W0(2);
% % % %         0 0 1 W0(3);
% % % %         0 0 0 1
% % % %     ];


        
    TETA1 = q(1);
    TETA2 = q(2);
    TETA3 = q(3);

    % Pose de chaque articulation
    T1 = T_Rot(O1,Z1,TETA1);
    T2 = T_Rot(O2,Z2,TETA2);
    T3 = T_Rot(O3,Z3,TETA3);

    Ts3 = T1*T2*T3;
    Tsw_cindir = Ts3*Tsw0; % NE PAS OUBLIER!!!!

    
        
        



end

