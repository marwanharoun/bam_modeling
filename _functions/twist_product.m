function V1V2 = twist_product(V1,V2)

    II = [
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0];
    
    V1V2 = (II*V1)'*V2;

end

