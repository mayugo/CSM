%%% Exemple sintonització analítica PI de 1r ordre
% Identificar el model, el diagrama de Bode, i implementar PI
% J.A.Mayugo, UdG, 2022

clear; close all;

%% Propietats del sistema
U  = 5;         % V, input 'bump test'
omega_ss= 104;  % rad/s
t0 = 300e-3;    % s
t1 = 375e-3;    % s

omega_n = 25;   % rad/s
zeta    = 1;    % críticament esmorteït

s = tf('s');
t_ = 0:0.001:0.6;    % resposta temporal

%% APARTAT a)
tau = t1-t0;
K = omega_ss/U;

G_m = K/(tau*s+1);

% Resposta en llaç obert:
U_ = U *ones(size(t_));
[omg,T,X]         = lsim(G_m,U_,t_);  % voltage/angular vel.
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló amb ' num2str(U) ' V al motor'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
saveas(H_ol,'./figures/Exemple_PI_1r_ordre_ol.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab i R-Locus
H_bode=figure
h = bodeplot(G_m); %,{0.1,20});
%setoptions(h,'MagUnits','abs','FreqScale','linear')

H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

%% APARTATS b) i c) Sintonització analítica d'un PI
% control velocitat llaç tancat considerant un sistema de 2n ordre
Kp = (2*zeta*omega_n*tau-1)/K 
Ki = omega_n^2*tau/K
PI = Kp + Ki/s              % controlador PI
G_cl = PI*G_m/(1+PI*G_m)    % model llaç tancat amb PI, fórmula de Mason
p_cl = pole(G_cl)

u = omega_ss;
u_ = u *ones(size(t_));
[omg_,T,X] = lsim(G_cl,u_,t_);  % voltage/angular vel.
H_cl=figure;
plot(t_,omg_,'LineWidth',1.8)
grid; title(['Resposta control PI, consigna ' num2str(u) ' rad/s'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24)
legend(["$K_p$ = " + num2str(round(Kp,2)) + ...
        ", $K_i$ = " + num2str(round(Ki,2))], ...
       'Interpreter','latex','FontSize',18,'Location','south');
ylim([0,120]);set(gca,'FontSize',18);legend boxoff
saveas(H_cl,'./figures/Exemple_PI_1r_ordre_cl.pdf')

%% APARTAT d) Error de variació de sortida: 'Error de velocitat'

e_v=1/(Ki*K); % rad/s2, error en la derivada de del valor de sortida

disp(["L'error estacionari és de "+num2str(round(e_v,4))+'rad/s2'])

%% Alternativa) Direct Syntesis Method
% control velocitat llaç tancat controlador PI
lambda_ = [10, 20, 50, 100];
u = omega_ss;
u_ = u *ones(size(t_));
for ii = 1:length(lambda_)
    Kp_(ii)= tau/K*lambda_(ii);
    Ti     = tau;
    
    PI2   = Kp_(ii)*(Ti*s+1)/(Ti*s); % controlador PI
    PI2   = Kp_(ii)*(1+1/(Ti*s)); % controlador PI
    G2_cl = PI2*G_m/(1+PI2*G_m);% model llaç tancat amb PI, fórmula de Mason
    p2_cl = pole(G2_cl);
    
    [omg_,T,X] = lsim(G2_cl,u_,t_);  % voltage/angular vel.
    OMG_(ii,:)=omg_;
end

H2_cl=figure;
hold on
legend_=string(lambda_);
for ii = 1:length(lambda_)
    plot(t_,OMG_(ii,:),'LineWidth',1.8);
    legend_(ii) = '$\lambda$ = ' + legend_(ii) + ', $K_p$ = ' + ...
                  num2str(round(Kp_(ii),2)) + ', $K_i$ = ' + ...
                  num2str(round(Kp_(ii)/Ti,2));
end
grid; title(['Resposta control PI, consigna ' num2str(u) ' rad/s'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',24);
legend(legend_,'Interpreter','latex','FontSize',18,'Location','south');
ylim([0,120]);set(gca,'FontSize',18);legend boxoff
saveas(H2_cl,'./figures/Exemple_PI_1r_ordre_2cl.pdf')
