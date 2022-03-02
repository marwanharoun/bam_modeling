function Mx = x(M)
    
    % Cette fonction trouve la matrice antisymetrique d'un vecteur de
    % dimention 3x1
    
    % Verification que M est soit 3x1 soit 1x3
    if size(M,1) ~= 3 || size(M,2) ~= 1
        assert(size(M,1) == 1 && size(M,2) == 3)
    end
    
    Mx = [    
        0 -M(3) M(2) ; 
        M(3) 0 -M(1) ; 
        -M(2) M(1) 0     
    ];



end

