function [THETA1,THETA2,THETA3,THETA4,THETA5,THETA6] = ikine_gen3lite(T,THETA6_guess,soltype,solnumber)

    D_we = 105+130;
    D_wo = 57;

    % Axe de chaque articulation
    Z1 = [0; 0; 1];
    Z2 = [0; -1; 0];
    Z3 = [0; 1; 0];
    
    Rd = [
        T(1,1) T(1,2) T(1,3);
        T(2,1) T(2,2) T(2,3);
        T(3,1) T(3,2) T(3,3);
        ];
    
    Ed = [T(1,4); T(2,4); T(3,4)];
    
    Xd = [T(1,1); T(2,1); T(3,1)];
    Yd = [T(1,2); T(2,2); T(3,2)];
    Zd = [T(1,3); T(2,3); T(3,3)];
    
    try
        W = Ed + D_wo*(sin(THETA6_guess)*Yd - cos(THETA6_guess)*Xd) - D_we*Zd;

        [THETA1,THETA2,THETA3] = ikine_anthroRRR(transpose(W),soltype);

        R1 = Rot(Z1,THETA1);
        R2 = Rot(Z2,THETA2);
        R3 = Rot(Z3,THETA3);

        R03 = R1*R2*R3;
        Rsw = R03\Rd;

        [THETA4,THETA5,THETA6] = dcm2eul(Rsw,solnumber);

        diff = THETA6 - THETA6_guess;

        if abs(diff) > 0.000000001
            [THETA1,THETA2,THETA3,THETA4,THETA5,THETA6] = ikine_gen3lite(T,THETA6,soltype,solnumber);
        end
    catch
        THETA1 = NaN;
        THETA2 = NaN;
        THETA3 = NaN;
        THETA4 = NaN;
        THETA5 = NaN;
        THETA6 = NaN;
    end
        

end

