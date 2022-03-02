function [THETA1,THETA2,THETA3] = ikine_anthroRRR(Wdes,soltype)
    
    Wdes = [Wdes(1),Wdes(2),Wdes(3)]
    assert(size(Wdes,1) == 1 && size(Wdes,2) == 3)
    assert(strcmp(soltype, 'rd') || strcmp(soltype, 'ru') || strcmp(soltype, 'ld') || strcmp(soltype, 'lu'))
    try

        L1 = 128.3+115;
        D1 = 30;
        L2 = 280;
        L3 = 140+105;

        Wx = Wdes(1);
        Wy = Wdes(2);
        Wz = Wdes(3);


        D = sqrt(Wx^2 + Wy^2 + (Wz-L1)^2 - D1^2);
        ALFA = atan2((Wz-L1),sqrt(Wx^2 + Wy^2 - D1^2));


        if strcmp(soltype, 'ru')
            % RIGHT UP:
            THETA1 = wrapangle(atan2(Wy,Wx) + asin(D1/sqrt(Wx^2+Wy^2)));
            % On ajoute -pi/2 a theta2 comparé aux notes
            THETA2 = wrapangle(ALFA + acos((L2^2 + D^2 - L3^2)/(2*D*L2))       -pi/2);
            % On prend le negatif de theta3 comparé au notes
            THETA3 = wrapangle(acos((D^2-L2^2-L3^2)/(2*L2*L3)));

        elseif strcmp(soltype, 'rd')
            % RIGHT DOWN:
            THETA1 = wrapangle(atan2(Wy,Wx) + asin(D1/sqrt(Wx^2+Wy^2)));
            THETA2 = wrapangle(ALFA - acos((L2^2 + D^2 - L3^2)/(2*D*L2))        -pi/2);
            THETA3 = wrapangle(-acos((D^2-L2^2-L3^2)/(2*L2*L3)));

        elseif strcmp(soltype, 'lu')
            % LEFT HIGH:
            THETA1 = wrapangle(atan2(Wy,Wx) - asin(D1/sqrt(Wx^2+Wy^2)) + pi);
            THETA2 = wrapangle(pi - ALFA - acos((L2^2 + D^2 - L3^2)/(2*D*L2))        -pi/2);
            THETA3 = wrapangle(-acos((D^2-L2^2-L3^2)/(2*L2*L3)));

        elseif strcmp(soltype, 'ld')
            % LEFT LOW:
            THETA1 = wrapangle(atan2(Wy,Wx) - asin(D1/sqrt(Wx^2+Wy^2)) + pi);
            THETA2 = wrapangle(pi - ALFA + acos((L2^2 + D^2 - L3^2)/(2*D*L2))        -pi/2);
            THETA3 = wrapangle(acos((D^2-L2^2-L3^2)/(2*L2*L3)));
        end
        [T,temp] = cindir_gen3_03([THETA1,THETA2,THETA3]);
        Wdes_exp = [T(1,4),T(2,4),T(3,4)]
        test = Wdes_exp - Wdes;
        assert(test(1)^2 < 0.00001 && test(1)^2 < 0.00001 && test(1)^2 < 0.00001)
    catch
        THETA1 = NaN;
        THETA2 = NaN;
        THETA3 = NaN;
    end

end

