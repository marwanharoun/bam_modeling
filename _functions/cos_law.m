function [c, ac, bc] = cos_law(a,b,ab)
%COS_LAW takes as inputs lengths a and b of two sides of a triangle, and
%the angle ab between them, and returns length c, and angles ac and bc

c = sqrt(a*a + b*b - 2*a*b*cos(ab));
ac = acos((a*a + c*c - b*b)/(2*a*c));
bc = acos((b*b + c*c - a*a)/(2*b*c));


end

