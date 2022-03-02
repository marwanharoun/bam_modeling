function [PHI,THETA,PSI]  = dcm2eul(R,solnumber)

    % Ceci est une fonction pour trouver les angle euler sous la forme ZXZ
    % seulement
    
%     clc;
    format long

    syms PHI THETA PSI
    assert(size(R,1) == 3 && size(R,2) == 3);
    assert(solnumber == 0 || solnumber == 1);
    
    
    % R_zxz_from_Rot = Rot([0 0 1],PHI)*Rot([1 0 0],THETA)*Rot([0 0 1], PSI);    
    R_eul_zxz = [
 
        [cos(PHI)*cos(PSI) - cos(THETA)*sin(PHI)*sin(PSI), - cos(PHI)*sin(PSI) - cos(PSI)*cos(THETA)*sin(PHI),  sin(PHI)*sin(THETA)]
        [cos(PSI)*sin(PHI) + cos(PHI)*cos(THETA)*sin(PSI),   cos(PHI)*cos(PSI)*cos(THETA) - sin(PHI)*sin(PSI), -cos(PHI)*sin(THETA)]
        [                             sin(PSI)*sin(THETA),                                cos(PSI)*sin(THETA),           cos(THETA)]

    ];

    a=R(3,3)
    if a < 0.9999 && a > -0.9999
        % Pour TETA positive:
        if solnumber == 0
            THETA = wrapangle(atan2(sqrt(R(3,2)^2+R(3,1)^2),R(3,3)));
            PHI = wrapangle(atan2(R(1,3),-R(2,3)));
            PSI = wrapangle(atan2(R(3,1),R(3,2)));
            
            R_test = eval(R_eul_zxz)
            R_minus_R1 = R - R_test

        % Pout TETA negative:
        elseif solnumber == 1
            THETA = wrapangle(-atan2(sqrt(R(3,2)^2+R(3,1)^2),R(3,3)));
            PHI = wrapangle(atan2(-R(1,3),R(2,3)));
            PSI = wrapangle(atan2(-R(3,1),-R(3,2)));
            
            R_test = eval(R_eul_zxz)
            R_minus_R2 = R - R_test
        end
    else
        THETA = 0;
        PSI = 0;
        PHI = wrapangle(acos(R(2,2)));
        
        R_test = eval(R_eul_zxz) 
        R_minus_R3 = R - R_test
    end

    

end

