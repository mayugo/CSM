%% Sol·lució moviment capçal amb XY taula: Moviment diagonal
% Generació de trajectòries
% 01_cin_09.tex
% J.A.Mayugo, UdG, 2022

close all; clear;
%% DADES
% -- dades del motor
n_max = 6460; % rpm
n_nom = 5060; % rpm
P_nom = 20;   % W
% -- dades del reductor 
i = 66;  
% -- dimensions politges
rG =  70e-3; % m, radi politges grans
rP =  40e-3; % m, radi politges petites
% Dades de l'encoder
np = 4;      % polsos per volta
Xcomp = 1;   % comptatge X1

% dades APARTAT b) de l'exercici
x_f =  200e-3   % m, desplaçament x final CANVIAR AQUEST VALOR PER OBTENIR UNA ALTRA SOLUCIÓ
y_f =  150e-3   % m, desplaçament y final CANVIAR AQUEST VALOR PER OBTENIR UNA ALTRA SOLUCIÓ
eps_max = 1000  % rad/s2, acceleració màxima del motor
t_total = 2.4   % segons, temps total moviment


%% Resolució APARTAT b) de l'exercici, marxa-moviment-parada
% Gir de cada motor per arribar a la posició final: cinemàtica inversa
i_t = i/rG;
theta1_f  = i_t*(+1*x_f + 1*y_f)  %  y = rG/i (theta_1 + theta2)
theta2_f  = i_t*(-1*x_f + 0*y_f)  %  x = rG/i (- theta2)

if abs(theta1_f) > abs(theta2_f)  
    % El motor 1 té el moviment més crític
    ddtheta1 = eps_max*sign(theta1_f); 
    t_b = t_total/2 - (ddtheta1^2*t_total^2-4*abs(ddtheta1)*abs(theta1_f) )^(1/2)/(2*abs(ddtheta1));
    dtheta1 = t_b * ddtheta1;

    % El motor 2 haurà de 'seguir' al motor 1 amb el mateix t_b
    ddtheta2 = (theta2_f)/(t_b*t_total-t_b^2);
    dtheta2 = t_b * ddtheta2;
else
    % El motor 2 té el moviment més crític
    ddtheta2 = eps_max*sign(theta2_f); 
    t_b = t_total/2 - (ddtheta2^2*t_total^2-4*abs(ddtheta2)*abs(theta2_f) )^(1/2)/(2*abs(ddtheta2));
    dtheta2 = t_b * ddtheta2;

    % El motor 1 haurà de 'seguir' al motor 2 amb el mateix t_b
    ddtheta1 = (theta1_f)/(t_b*t_total-t_b^2);
    dtheta1 = t_b * ddtheta1;
end

% Definir trams de moviment: marxa-moviment-parada
t_abs1(1) = t_b;
t_abs1(2) = t_abs1(1) + t_total-2*t_b;
t_abs1(3) = t_abs1(2) + t_b;

theta1_a(1) =        + 1/2*ddtheta1*t_abs1(1)^2;
theta1_a(2) = theta1_a(1) + dtheta1*(t_abs1(2)-t_abs1(1));
theta1_a(3) = theta1_a(2) + dtheta1*(t_abs1(3)-t_abs1(2)) - 1/2*ddtheta1*(t_abs1(3)-t_abs1(2))^2;
theta1_t = theta1_a(3);

theta2_a(1) =        + 1/2*ddtheta2*t_abs1(1)^2;
theta2_a(2) = theta2_a(1) + dtheta2*(t_abs1(2)-t_abs1(1));
theta2_a(3) = theta2_a(2) + dtheta2*(t_abs1(3)-t_abs1(2)) - 1/2*ddtheta2*(t_abs1(3)-t_abs1(2))^2;
theta2_t = theta2_a(3);

disp('RESULTATS DE APARTAT b) DE L EXERCICI 1.1')
disp(' ')
disp('Motion control del motor 1')
%  Tram & tipus & $t$ [s] & $\theta_1$ [rad] & $\dot{\theta}$ [rad/s] & $\ddot{\theta}$ [rad/s\tss{2}]]    \\
disp(['Tram 1 , 0 a 1, acceleració',',   t_1 =',num2str(t_b,'%10.3f'),'s ,     theta1_1 =',num2str(theta1_a(1),'%10.3f'),'rad ,  eps1_max =',num2str(ddtheta1,'%10.3f') 'rad/s2' ])
disp(['Tram 2 , 1 a 2, vel. constant',', t_2 =',num2str(t_total-2*t_b,'%10.3f'),'s ,     theta1_2 =',num2str(theta1_a(2)-theta1_a(1),'%10.3f'),'rad , omg1_max =',num2str(dtheta1,'%10.3f') 'rad/s' ])
disp(['Tram 3 , 2 a 3, desacceleració',',t_3 =',num2str(t_b,'%10.3f'),'s ,     theta1_3 =',num2str(theta1_a(3)-theta1_a(2),'%10.3f'),'rad , eps1_max =',num2str(-ddtheta1,'%10.3f') 'rad/s2' ])
disp(' ')
disp(['temps total = ',num2str(t_total,'%10.2f'),'s , theta1 total recorreguda = ',num2str(abs(theta1_t),'%10.2f'), 'rad'])
disp(' ')

disp('Motion control del motor 2')
%  Tram & tipus & $t$ [s] & $\theta_2$ [rad] & $\dot{\theta}$ [rad/s] & $\ddot{\theta}$ [rad/s\tss{2}]]    \\
disp(['Tram 1 , 0 a 1, acceleració',',   t_1 =',num2str(t_b,'%10.3f'),'s ,     theta2_1 =',num2str(theta2_a(1),'%10.3f'),'rad ,  eps2_max =',num2str(ddtheta2,'%10.3f') 'rad/s2' ])
disp(['Tram 2 , 1 a 2, vel. constant',', t_2 =',num2str(t_total-2*t_b,'%10.3f'),'s ,     theta2_2 =',num2str(theta2_a(2)-theta2_a(1),'%10.3f'),'rad , omg2_max =',num2str(dtheta2,'%10.3f') 'rad/s' ])
disp(['Tram 3 , 2 a 3, desacceleració',',t_3 =',num2str(t_b,'%10.3f'),'s ,     theta2_3 =',num2str(theta2_a(3)-theta2_a(2),'%10.3f'),'rad , eps2_max =',num2str(-ddtheta2,'%10.3f') 'rad/s2' ])
disp(' ')
disp(['temps total = ',num2str(t_total,'%10.2f'),'s , theta2 total recorreguda = ',num2str(abs(theta2_t),'%10.2f'), 'rad'])
disp(' ')

% Obtenir la trajectoria discretitzada posicio, velocitat i acceleració dels motors
t_ = [0:0.005:t_total];
theta1_ = zeros(1,length(t_));
dtheta1_ = zeros(1,length(t_));
ddtheta1_ = zeros(1,length(t_));
for ii = [1:length(t_)]
    if  t_(ii) <= t_abs1(1)
        theta1_(ii) = + 1/2*ddtheta1*t_(ii)^2;
        dtheta1_(ii) = ddtheta1*t_(ii);
        ddtheta1_(ii) = ddtheta1;
    elseif  t_(ii) <= t_abs1(2)
        theta1_(ii) = theta1_a(1) + dtheta1*(t_(ii)-t_abs1(1));
        dtheta1_(ii) = dtheta1;
        ddtheta1_(ii) = 0;
    else %if  t_(ii) <= t_i(3)
        theta1_(ii) = theta1_a(2) + dtheta1*(t_(ii)-t_abs1(2)) - 1/2*ddtheta1*(t_(ii)-t_abs1(2))^2;
        dtheta1_(ii) = dtheta1 - ddtheta1*(t_(ii)-t_abs1(2));
        ddtheta1_(ii) = -ddtheta1;
    end
end

theta2_ = zeros(1,length(t_));
dtheta2_ = zeros(1,length(t_));
ddtheta2_ = zeros(1,length(t_));
for ii = [1:length(t_)]
    if  t_(ii) <= t_abs1(1)
        theta2_(ii) = + 1/2*ddtheta2*t_(ii)^2;
        dtheta2_(ii) = ddtheta2*t_(ii);
        ddtheta2_(ii) = ddtheta2;
    elseif  t_(ii) <= t_abs1(2)
        theta2_(ii) = theta2_a(1) + dtheta2*(t_(ii)-t_abs1(1));
        dtheta2_(ii) = dtheta2;
        ddtheta2_(ii) = 0;
    else %if  t_(ii) <= t_i(3)
        theta2_(ii) = theta2_a(2) + dtheta2*(t_(ii)-t_abs1(2)) - 1/2*ddtheta2*(t_(ii)-t_abs1(2))^2;
        dtheta2_(ii) = dtheta2 - ddtheta2*(t_(ii)-t_abs1(2));
        ddtheta2_(ii) = -ddtheta2;
    end
end

% Representar posició, velocitat i acceleració dels motors
H1=figure;font_size=18;
subplot(3,1,1);
plot(t_,theta1_,'LineWidth',2)
ylabel('${\theta_1}$ [rad]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,dtheta1_,'LineWidth',2)
ylabel('$\dot{\theta}_1$ [rad/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,ddtheta1_,'LineWidth',2)
ylabel('$\ddot{\theta}_1 \; \rm{[rad/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H1,'./figures/CIN_taulaXY_theta1.svg')

H2=figure;font_size=18;
subplot(3,1,1);
plot(t_,theta2_,'LineWidth',2)
ylabel('${\theta_2}$ [rad]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,dtheta2_,'LineWidth',2)
ylabel('$\dot{\theta}_2$ [rad/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,ddtheta2_,'LineWidth',2)
ylabel('$\ddot{\theta}_2 \; \rm{[rad/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H2,'./figures/CIN_taulaXY_theta2.svg')

% Comprovació trajectòria capçal: fent ús de la cinemàtica directe
Ht=figure;font_size=18;
plot((0*theta1_-theta2_)/i_t,(theta1_+theta2_)/i_t,'LineWidth',2)
xlabel('${x}$ [m]','Interpreter','latex','FontSize',font_size)
ylabel('${y}$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
axis equal
saveas(Ht,'./figures/CIN_taulaXY_XY.svg')

% Representar posició, velocitat i acceleració del capçal en coordenades XY
H3=figure;font_size=18;
subplot(3,1,1);
plot(t_,(0*theta1_-theta2_)/i_t,'LineWidth',2)
ylabel('${x}$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,(0*dtheta1_-dtheta2_)/i_t,'LineWidth',2)
ylabel('$\dot{x}$ [m/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,(0*ddtheta1_-ddtheta2_)/i_t,'LineWidth',2)
ylabel('$\ddot{x} \; \rm{[m/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H3,'./figures/CIN_taulaXY_X.svg')

H4=figure;font_size=18;
subplot(3,1,1);
plot(t_,(theta1_+theta2_)/i_t,'LineWidth',2)
ylabel('${y}$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,(dtheta1_+dtheta2_)/i_t,'LineWidth',2)
ylabel('$\dot{y}$ [m/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,(ddtheta1_+ddtheta2_)/i_t,'LineWidth',2)
ylabel('$\ddot{y} \; \rm{[m/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H4,'./figures/CIN_taulaXY_Y.svg')