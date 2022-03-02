% Function which returns the rotation matrix of angle teta around vector u
% !!! Make sure u is a unit vector !!!

function Rot = Rot(U,TETA)

  
    Rot=[
        
        U(1)^2 + (1-U(1)^2)*cos(TETA)               ,   U(1)*U(2)*(1-cos(TETA)) - U(3)*sin(TETA)    ,   U(1)*U(3)*(1-cos(TETA)) + U(2)*sin(TETA);
        U(1)*U(2)*(1-cos(TETA)) + U(3)*sin(TETA)    ,   U(2)^2 + (1-U(2)^2)*cos(TETA)               ,   U(2)*U(3)*(1-cos(TETA)) - U(1)*sin(TETA);
        U(1)*U(3)*(1-cos(TETA)) - U(2)*sin(TETA)    ,   U(2)*U(3)*(1-cos(TETA)) + U(1)*sin(TETA)    ,   U(3)^2 + (1-U(3)^2)*cos(TETA)
      
        ];
end