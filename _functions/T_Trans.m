function T = T_Trans(U,D)

U = [U(1); U(2); U(3)];
I = [1 0 0;0 1 0;0 0 1];
T = [[I;0 0 0] [D*U;1]];


end

