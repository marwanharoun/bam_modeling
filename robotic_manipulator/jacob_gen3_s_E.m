function Je = Je(q)

    % Ne pas oublier que multiplier la jacobienne geometrique par la matrice des
    % vitesses angulaires donne le torseur et vitesse lineaire d'un point
    % attaché au repère {e} de l'effecteur mais qui coincide instantanement
    % avec le centre du repère {s}. Pour trouver le torseur d'un point qui
    % coincide instantanement avec le centre du repere {e} de l'effecteur,
    % ou plutôt pour avoir la jacobienne "utile" qui est normalement
    % associée à un manipulateur, il faut faire un changement de point de representation (5.2.2, Robotique)
    
    % Alternativement, dans le calcul des visseurs plus bas, on peut
    % remplacer le vecteur O1O, O2O, etc... par O1E, O2E, etc... où E est
    % le centre de l'effecteur (mais non-pas au repos). On peut avoir les
    % coordonnées de E à partir de Tse (dernière colonne).

    
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
    
    
    % Vecteur unitaire à q=0 de chaque articulation (donc manipulateur au repos)
    Z1 = [0; 0; 1];
    Z2 = [0; -1; 0];
    Z3 = [0; 1; 0];
    Z4 = [0; 0; 1];
    Z5 = [1; 0; 0];
    Z6 = [0; 0; 1];

    
    % Centre du repère de chaque articulation au repos
    O1 = [0;             0        ;        0                               ];
    O2 = [0;             0        ;        115+128.3                       ];
    O3 = [0;             -30      ;        115+128.3+280                   ];
    O4 = [0;             -30+20   ;        115+128.3+280                   ];
    O5 = [0;             -30+20   ;        115+128.3+280+140+105           ];
    O6 = [28.5*2;        -30+20   ;        115+128.3+280+140+105           ];
    E0 = [28.5*2;        -30+20   ;        115+128.3+280+140+105+105+130   ];

    % Pose de l'éffecteur en configuration de repos
    Tse0 = [
        1 0 0 E0(1);
        0 1 0 E0(2);
        0 0 1 E0(3);
        0 0 0 1
    ];
    
    % Visseurs de chaque articulation au repos
    V1 = [Z1;-cross(Z1,O1)];
    V2 = [Z2;-cross(Z2,O2)];
    V3 = [Z3;-cross(Z3,O3)];
    V4 = [Z4;-cross(Z4,O4)];
    V5 = [Z5;-cross(Z5,O5)];
    V6 = [Z6;-cross(Z6,O6)];

    % Nombre de configurations en entrée
    n = size(q,1);
    
    for i =1:n
        
        TETA1 = q(i,1);
        TETA2 = q(i,2);
        TETA3 = q(i,3);
        TETA4 = q(i,4);
        TETA5 = q(i,5);
        TETA6 = q(i,6);        
        
        % Pose de chaque articulation (mais maintenant qui depend de q, et pas au repos)
        T1 = T_Rot(O1,Z1,TETA1);
        T2 = T_Rot(O2,Z2,TETA2);
        T3 = T_Rot(O3,Z3,TETA3);
        T4 = T_Rot(O4,Z4,TETA4);
        T5 = T_Rot(O5,Z5,TETA5);
        T6 = T_Rot(O6,Z6,TETA6);


        % Colonnes de la Jacobienne. NE PAS OUBLIER AdT !!!
        J1 = V1;
        J2 = AdT(T1)*V2;
        J3 = AdT(T1*T2)*V3;
        J4 = AdT(T1*T2*T3)*V4;
        J5 = AdT(T1*T2*T3*T4)*V5;
        J6 = AdT(T1*T2*T3*T4*T5)*V6;


        % La matrice suivante est la jacobienne à un point coincidant
        % instatanement avec O. 
        J = [J1 J2 J3 J4 J5 J6];


        % Pour trouver la matrice à un point coincidant instantanement avec E,
        % voir 5.28. 

        % Coordonnées de E:
        Tse = T1*T2*T3*T4*T5*T6*Tse0;
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
        Je(:,:,i) = M*J;
    end



end

