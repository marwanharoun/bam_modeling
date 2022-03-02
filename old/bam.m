clc
clear

syms a b c alfa beta

% Description:
% Triangle ABC with angles alfa at A, beta at B, gamma at C
% Prismatic joints between A, and B and C respectively
% Revolute joints between BC, and B and C respectively
% AB = b, AC = a, BC = c


alfa_min = 70*pi/180;
alfa_max = 120*pi/180;
c = 40;
A = [0,0,0,0];
a_min = 20;
a_max = 50;

% Generate x,y for point A as the as AC varies between a_min and a_max, for values of alfa between alfa_min and alfa_max
for alfa = alfa_min:5*pi/180:alfa_max
    for a = a_min:5:a_max
        syms b
        % Cosine law for alfa:
        eq_alfa = c^2 - a^2 - b^2 + 2*a*b*cos(alfa) == 0;
        b_sol = solve(eq_alfa,b);
        b = b_sol(2);

        % Cosine law for beta:
        eq_beta = a^2 - b^2 - c^2 + 2*b*c*cos(beta) == 0;
        beta_sol = solve(eq_beta,beta);
        beta_sol = abs((beta_sol(1)));

        x = b*cos(beta_sol);
        y = b*sin(beta_sol);

        % Array for generated points A
        A = [A; x y alfa beta_sol];

    end
end

for i = 1:7:length(A)
    plot3(A((i+1):(i+7),3)*180/pi,A((i+1):(i+7),1),A((i+1):(i+7),2),'-o','LineWidth',1)
    hold on
end
grid on
axis([0 180 0 c 0 c])


