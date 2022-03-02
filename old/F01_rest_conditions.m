function [PHI2, PHI5, l12, l123, Os, O1, O2, Oe, P1, P2, P3, Zs, Z1, X1, Z2] = F01_rest_conditions(l1,l2,l3,l4,d5,TETA1,PHI1,PHI3)

    % PHI1 is the angle between the diameter of the cup alligned with the point
    %   of contact, and the diameter of the cup coplanar with the red arm (see
    %   figure in notes) at rest.
    % PHI2 is the angle between the cup and the line passing between Os and O1
    %   l12 is the distance between Os and O1.

    % Nomenclature example: 
    % Ref_a1 is frame {1} of leg (a) expressed in the {s} frame.
    % Ref_a2_a1 is frame {2} of leg (a) (also written {2a}) expressed in
    %   frame {1} of leg (a) (also written {1a}).
    % O_2 is the coordinates of O_2 expressed in the {s} frame.
    % O_2_1 is the coordinates of O_2 expressed in the {1} frame.

    % -----------------------------------------------------------------------
    %                         FIXED GEOMETRIES
    % -----------------------------------------------------------------------
    PHI2 = atan2(l2,l1);
    l12 = sqrt(l2*l2 + l1*l1);

    PHI5 = atan2(l2+l3,l1);
    l123 = sqrt(l1*l1 + (l2+l3)^2);

    % -----------------------------------------------------------------------
    %                         FRAMES OF (a) AT REST
    % -----------------------------------------------------------------------
    % Frame {s} at rest position:
    Ref_s = [[1;0;0] [0;1;0] [0;0;1]];

    % Frame {1} at rest position:
    Ref_a1 = simplify(Rot(Ref_s(:,3), TETA1)*Ref_s);

    % Frame {2} at rest position:
    Ref_a2 = simplify(Rot(Ref_a1(:,2), PHI1)*Ref_a1);
    Ref_a2 = simplify(Rot(Ref_a2(:,3), PHI3)*Ref_a2);
    Ref_a2 = [Ref_a2(:,2) Ref_a2(:,3) Ref_a2(:,1)];

    % Frame {2} at rest position expressed in coordinates of {1}:
    Ref_a2_a1 = simplify(Rot(Ref_s(:,2), PHI1)*Ref_s);
    Ref_a2_a1 = simplify(Rot(Ref_a2_a1(:,3), PHI3)*Ref_a2_a1);
    Ref_a2_a1 = [Ref_a2_a1(:,2) Ref_a2_a1(:,3) Ref_a2_a1(:,1)];

    % -----------------------------------------------------------------------
    %                         ORIGINS OF (a) AT REST
    % -----------------------------------------------------------------------
    Os = [0;0;0];
    O1 = [l12*cos(PHI2+TETA1);l12*sin(PHI2+TETA1);0];

    % Attitude of {1} with respect to {s} at rest:
    T_1_s = [[Ref_a1;0 0 0] [O1;1]];

    % Coordinate of O2 in frame {1}: 
    % (It's different from the one found in my notes. Verify)
    O2_1 = [l4*cos(PHI3)*cos(PHI1);l4*sin(PHI3)+l3;l4*cos(PHI3)*-sin(PHI1)];

    % Coordinate of O2 in frame {s}: 
    O2 = T_1_s*[O2_1;1];
    O2 = O2(1:3);
    % Attitude of {2} with respect to {1} at rest:
    T_2_1 = [[Ref_a2_a1;0 0 0] [O2_1;1]];

    % Coordinate of Oe in frame {2}: 
    Oe_2 = [0;0;d5];

    % Coordinate of Oe in frame {s}: 
    Oe = T_1_s*T_2_1*[Oe_2;1];
    Oe = Oe(1:3);

    % -----------------------------------------------------------------------
    %                         OTHER POINTS OF (a) AT REST
    % -----------------------------------------------------------------------
    P1 = [l1*cos(TETA1);l1*sin(TETA1);0];
    P2 = [2*l1*cos(TETA1);2*l1*sin(TETA1);0];
    P3 = [l123*cos(PHI5+TETA1);l123*sin(PHI5+TETA1);0];


    % -----------------------------------------------------------------------
    %                             AXES OF (a) AT REST
    % -----------------------------------------------------------------------
    Zs = Ref_s(:,3);
    Z1 = Ref_a1(:,3);
    X1 = Ref_a1(:,1);
    Z2 = Ref_a2(:,3);


end

