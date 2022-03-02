function T = T_Rot(O, W, TETA)
%T_ROT Summary of this function goes here
%   Detailed explanation goes here

I = [1 0 0;0 1 0;0 0 1];
W = [W(1); W(2); W(3)];
O = [O(1); O(2); O(3)];
R = Rot(W,TETA);
T = [[R;0 0 0] [(I-R)*O;1]];












end

