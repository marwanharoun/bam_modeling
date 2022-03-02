function Tse_cindir = cindir_gen3(q)

    % Axe de chaque articulation
    Z1 = [0; 0; 1];
    Z2 = [0; -1; 0];
    Z3 = [0; 1; 0];
    Z4 = [0; 0; 1];
    Z5 = [1; 0; 0];
    Z6 = [0; 0; 1];

    % Centre du repère de chaque articulation à la configuration de repos
    O1 = [0;             0        ;        0                               ];
    O2 = [0;             0        ;        115+128.3                       ];
    O3 = [0;             -30      ;        115+128.3+280                   ];
    O4 = [0;             -30      ;        115+128.3+280                   ];
    O5 = [0;             -30      ;        115+128.3+280+140+105           ];
    O6 = [28.5*2;        -30      ;        115+128.3+280+140+105           ];
    E0 = [28.5*2;        -30      ;        115+128.3+280+140+105+105+130   ];


    % Pose de l'éffecteur en configuration de repos
    Tse0 = [
        1 0 0 E0(1);
        0 1 0 E0(2);
        0 0 1 E0(3);
        0 0 0 1
    ];

    % Nombre de configurations en entrée
    n = size(q,1);
    
    for i = 1:n
        
        TETA1 = q(i,1);
        TETA2 = q(i,2);
        TETA3 = q(i,3);
        TETA4 = q(i,4);
        TETA5 = q(i,5);
        TETA6 = q(i,6);
        
        % Pose de chaque articulation
        T1 = T_Rot(O1,Z1,TETA1);
        T2 = T_Rot(O2,Z2,TETA2);
        T3 = T_Rot(O3,Z3,TETA3);
        T4 = T_Rot(O4,Z4,TETA4);
        T5 = T_Rot(O5,Z5,TETA5);
        T6 = T_Rot(O6,Z6,TETA6);

        Tse_cindir(:,:,i) = T1*T2*T3*T4*T5*T6*Tse0; % NE PAS OUBLIER!!!!
        
        
    end


end

