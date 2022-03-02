function J = F03_jacobian(Os, O1, O2, Zs, Z1, X1, Z2, Ts, T11, T12, T2)

    % -----------------------------------------------------------------------
    %                          KINEMATIC WRENCHES AT REST
    % -----------------------------------------------------------------------
    Vs = [Zs;-cross(Zs,Os)];
    V11 = [Z1;-cross(Z1,O1)];
    V12 = [X1;-cross(X1,O2)];
    V2 = [[0;0;0];Z2];


    % -----------------------------------------------------------------------
    %                          KINEMATIC WRENCHES NOT AT REST
    % -----------------------------------------------------------------------
    Js = Vs;
    J11 = simplify(AdT(Ts)*V11);
    J12 = simplify(AdT(Ts*T11)*V12);
    J2 = simplify(AdT(Ts*T11*T12)*V2);
    
    J = [Js J11 J12 J2];
end

