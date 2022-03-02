function Tse_dh = cindir_gen3_dh(q)

    % Nombre de configurations en entrée
    n = size(q,1);
    
    for i = 1:n
        
        TETA1 = q(i,1);
        TETA2 = q(i,2);
        TETA3 = q(i,3);
        TETA4 = q(i,4);
        TETA5 = q(i,5);
        TETA6 = q(i,6);

        % Changement de repères
        T12 = T_DH(TETA1             ,pi/2        ,128.3+115,0);
        T23 = T_DH(TETA2 + pi/2      ,pi          ,30,280);
        T34 = T_DH(TETA3 + pi/2      ,pi/2        ,20,0);
        T45 = T_DH(TETA4 + pi/2      ,pi/2        ,140+105,0);
        T56 = T_DH(TETA5 + pi        ,pi/2        ,28.5*2,0);
        T6e = T_DH(TETA6 + pi/2      ,0           ,105+130,0);

        % Don't forget to adjust TETA's to account for initial frame rotation with
        % respect to previous frame (before moving manipulator from zero condition)

        Tse_dh(:,:,i) = T12*T23*T34*T45*T56*T6e;

    
    end


end

