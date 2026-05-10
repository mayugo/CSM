%%% Exemple sintonització analítica PD de 2n ordre
% Identificar el model, el diagrama de Bode, i implementar PI
% J.A.Mayugo, UdG, 2022

clear; close all;

%% Propietats del sistema
s = tf('s');
G_m = 150/((1+0.15*s)*s);

e_ss = 0;   % eu, error en règim estacionari nul
SP = 5;     % %,  valor del sobre pic (SP) inferior a 5%
ts2 = 0.57; % s,  temps per error inferior al 2%

U = 1;
t_ = 0:0.01:1;

% Resposta en llaç obert:
U_ = U *ones(size(t_));
[omg,T,X]         = lsim(G_m,U_,t_);  % voltage/angular vel.
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló amb ' num2str(U) ' V al motor'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
saveas(H_ol,'./figures/Exemple_PD_2n_ordre_ol.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab i R-Locus
H_bode=figure
h = bodeplot(G_m); %,{0.1,20});
%setoptions(h,'MagUnits','abs','FreqScale','linear')

H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

%% APARTA b) i c) Sintonització analítica d'un PD
% control velocitat llaç tancat

A = ((log(100) - log(SP))/pi)^2
zeta = (A/(1+A))^0.5   % factor d'esmorteïment
omega_n = 4/(zeta*ts2) % rad/s

Kp = omega_n^2/1000 
Ki = 0
Kd = (2*zeta*omega_n-10)/1000

%Kp = 0.1034
%Kd = 0.0040
PID = Kp + Ki/s + Kd*s        % controlador PID
G_cl = PID*G_m/(1+PID*G_m)    % model llaç tancat amb PID, fórmula de Mason
p_cl = pole(G_cl)
z_cl = zero(G_cl)

E = 1/(1+PID*G_m)

% Model de segon ordre pur
G_cli = 1000*Kp/(s^2+(10+1000*Kd)*s+1000*Kp)
%G_cli = PID_G_mi/(1+PID_G_mi)  % model llaç tancat amb PID, fórmula de Mason
p_cli = pole(G_cli)
z_cli = zero(G_cli)

u = 1;
u_ = u *ones(size(t_));
u05_ = 1.05*u_;
u02_ = 1.02*u_;

[omg_,T,X]         = lsim(G_cl,u_,t_);  % voltage/angular vel.
[omgi_,T,X]        = lsim(G_cli,u_,t_);  % voltage/angular vel.
H_cl=figure;hold on;
plot(t_,omg_,'LineWidth',1.8)
plot(t_,omgi_,'LineWidth',1.8)
plot(t_,u02_)
plot(t_,u05_)
plot([ts2 ts2],[0 1.02])
grid; title(['Resposta control PD, consigna ' num2str(u) ' ue'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$UE$ [ue]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
saveas(H_cl,'./figures/Exemple_PD_2n_ordre_cl.pdf')