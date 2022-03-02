function [T1, T2, T3, T4] = F02_transformation_matrices(Os, O1, TETA1, TETA2, TETA3, l4, Zs, Z1, X1, Z2)
    
    % -----------------------------------------------------------------------
    %              DISPLACEMENT MATRICES (DEPENDANT ON q) (IN {s})
    % -----------------------------------------------------------------------
    T1 = T_Rot(Os,Zs,TETA1);
    T2 = T_Rot(O1,Z1,TETA2);
    T3 = T_Rot(O1,X1,TETA3);
    T4 = T_Trans(Z2,l4);
    


end