clear
clc

syms l1 l2 l3 l4 d5 TETA1 TETA2 TETA3 PHI1 PHI3


[PHI2, PHI5, l12, l123, Os, O1, O2, Oe, P1, P2, P3, Zs, Z1, X1, Z2] = F01_rest_conditions(l1,l2,l3,l4,d5,TETA1,PHI1,PHI3);



[T1, T2, T3, T4] = F02_transformation_matrices(Os, O1, TETA1, TETA2, TETA3, l4, Zs, Z1, X1, Z2);

% Attitude of the end effector at rest:
Te0 = [
    1 0 0 Oe(1);
    0 1 0 Oe(2);
    0 0 1 Oe(3);
    0 0 0 1
];

Te = T1*T2*T3*T4*Te0;

% J = F03_jacobian(Os, O1, O2, Zs, Z1, X1, Z2, T1, T2, T3, T4);

A01_rest_conditions_num