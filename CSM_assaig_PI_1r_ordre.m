%%% Exercici sintonització analítica PI de 1r ordre
% Identificar el model, el diagrama de Bode, i implementar PI
% J.A.Mayugo, UdG, 2023

clear; close all;

%% Propietats del sistema
U  = 48;        % V, input 'bump test'
A  = 2.20;      % rad/sV
B  = 12.5e-3;   % s

omega_n = 80;   % rad/s
zeta    = 0.90; % críticament esmorteït

s = tf('s');
t_ = 0:0.001:0.1;    % resposta temporal

%% APARTAT b)
tau = B;
K = A;

G_m = K/(tau*s+1);

% Resposta en llaç obert:
U_ = U *ones(size(t_));
[omg,T,X]         = lsim(G_m,U_,t_);  % voltage/angular vel.
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló amb ' num2str(U) ' V al motor'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
%saveas(H_ol,'./figures/Exemple_PI_1r_ordre_ol.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab i R-Locus
H_bode=figure
h = bodeplot(G_m); %,{0.1,20});
%setoptions(h,'MagUnits','abs','FreqScale','linear')

H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

% control velocitat llaç tancat considerant un sistema de 2n ordre
Kp = (2*zeta*omega_n*tau-1)/K 
Ki = omega_n^2*tau/K
PI = Kp + Ki/s              % controlador PI
G_cl = PI*G_m/(1+PI*G_m)    % model llaç tancat amb PI, fórmula de Mason
p_cl = pole(G_cl)

u = 5400 ;
u_ = u/30*pi *ones(size(t_));
[omg_,T,X] = lsim(G_cl,u_,t_);  % voltage/angular vel.
H_cl=figure;
plot(t_,omg_,'LineWidth',1.8)
grid; title(['Resposta control PI, consigna ' num2str(u) ' rpm'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24)
legend(["$K_p$ = " + num2str(round(Kp,2)) + ...
        ", $K_i$ = " + num2str(round(Ki,2))], ...
       'Interpreter','latex','FontSize',18,'Location','south');
%ylim([0,120]);
set(gca,'FontSize',18);legend boxoff
%saveas(H_cl,'./figures/Exemple_PI_1r_ordre_cl.pdf')


