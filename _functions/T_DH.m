function T_DH = T_DH(TETA,ALFA,D,A)
T_DH = [

    cos(TETA),              -sin(TETA)*cos(ALFA),   sin(TETA)*sin(ALFA),    A*cos(TETA); 
    sin(TETA),              cos(TETA)*cos(ALFA),    -cos(TETA)*sin(ALFA),   A*sin(TETA);
    0,                      sin(ALFA),              cos(ALFA),              D;
    0,                      0,                      0,                      1

];
end

