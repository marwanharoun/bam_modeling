clear;
clc;

format bank;
rng(1234);


% exemple de sequence de configurations
configvars =[0.0,0.0,0.0,0.0,0.0,0.0;
             pi/4,pi/5,pi/6,pi/7,pi/8,pi/5;
             12,5,6.7,0.0,42.1,pi/3.5;
             2*pi*rand(2,6)]
nconfigs = size(configvars,1);
         
disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp('TEST DES FONCTIONS CR��ES')
disp('-------------------------')
% votre fonction sera testée avec un tableau k*6 configvars différent
% mais devrait pouvoir prendre un tel tableau et retourner 
% un tableau Ts de taille 4x4xk
Ts = cindir_gen3(configvars);

% si ce test ne passe pas, votre prototype de function n'est pas bon
assert(size(Ts,1) == 4 && size(Ts,2) == 4 && size(Ts,3) == nconfigs)

% cette fonction devra retourner essentiellement le même résultat,
% mais en utilisant les paramètres DH
Ts_dh = cindir_gen3_dh(configvars);

error = 0;
for k=1:nconfigs
    error = error+norm(Ts(:,:,k)-Ts_dh(:,:,k));
end
disp(['Difference entre les deux méthodes: ',num2str(error)])

Js = jacob_gen3_s_E(configvars)
% si ce test ne passe pas, votre prototype de function n'est pas bon
assert(size(Js,1) == 6 && size(Js,2) == 6 && size(Js,3) == nconfigs)

% Il est suggéré de vérifier aussi avec les résultats obtenus
% avec la Robotics Toolbox de Peter Corke

disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp(' ')
disp('TEST SELON ROBOTICS TOOLBOX')
disp('---------------------------')


L(1) = Link('revolute', 'd', 128.3+115, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 30, 'a', 280, 'alpha', pi, 'offset', pi/2);
L(3) = Link('revolute', 'd', 20, 'a', 0, 'alpha', pi/2, 'offset', pi/2);
L(4) = Link('revolute', 'd', 140+105, 'a', 0, 'alpha', pi/2, 'offset', pi/2);
L(5) = Link('revolute', 'd', 28.5*2, 'a', 0, 'alpha', pi/2, 'offset', pi);
L(6) = Link('revolute', 'd', 105+130, 'a', 0, 'alpha', 0, 'offset', pi/2);


Gen3lite = SerialLink(L,'name','Gen3lite');
Gen3lite % affiche les parametres DH et autres informations sur le robot 
% Gen3lite.plot([pi/3 pi/4 pi/3 0 pi/4 0]) % pour afficher un modele graphique du robot
% % Gen3lite.fkine([0 0 0 0 0 0])
% Gen3lite.jacob0([pi/3 pi/4 pi/3 0 pi/4 0]) % calcule la jacobienne J_E^s(q) - attention, l?ordre des lignes est different



for i = 1:size(configvars,1)
    % Gen3lite.plot([0 0 0 0 0 0]) % pour afficher un modele graphique du robot
    % Gen3lite.fkine([0 0 0 0 0 0])
    Js_corke(:,:,i) = Gen3lite.jacob0(configvars(i,:)); % calcule la jacobienne J_E^s(q) - attention, l?ordre des lignes est different
    Js_corke(:,:,i)
end