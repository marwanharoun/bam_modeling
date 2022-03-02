function AdT = AdT(T)

    % Verification que T est effectivement 4x4
    assert(size(T,1) == 4 && size(T,2) == 4)

    l = [

        T(1,4);
        T(2,4);
        T(3,4);

        ];



    lx = x(l);



    R = [

        T(1,1) T(1,2) T(1,3);
        T(2,1) T(2,2) T(2,3);
        T(3,1) T(3,2) T(3,3);

        ];



    ZERO = [

        0 0 0;
        0 0 0;
        0 0 0

        ];


    AdT = [

        R,ZERO;
        lx*R,R

        ];




end

