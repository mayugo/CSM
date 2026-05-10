%%% Exemple sintonització Direct Syntesis Method
% Implementar PI a un sistema de 1r ordre
% J.A.Mayugo, UdG, 2022

clear; close all;

%% Propietats del sistema
s   = tf('s');
tau = 0.05;
K   = 300/24;

G_m = K/(tau*s+1);

% Resposta en llaç obert:
t_ = 0:0.001:0.6;    % resposta temporal
U_ = ones(size(t_));
[omg,T,X] = lsim(G_m,U_,t_);  
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló unitària'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('Resposta [eu]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
saveas(H_ol,'./figures/Exemple_PI_1r_ordre_ol.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab i R-Locus
H_bode=figure
h = bodeplot(G_m); %,{0.1,20});
%setoptions(h,'MagUnits','abs','FreqScale','linear')

H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

%G_m = K/1.5/(tau*s*1.5+1);


%% APARTAT: Direct Syntesis Method
% control llaç tancat controlador PI
lambda_ = [10, 20, 50, 100];
u = 288;
u_ = u *ones(size(t_));
for ii = 1:length(lambda_)
    Kp_(ii)= tau/K*lambda_(ii);
    Ti     = tau;
    
    PI2   = Kp_(ii)*(Ti*s+1)/(Ti*s); % controlador PI
    PI2   = Kp_(ii)*(1+1/(Ti*s)); % controlador PI
    G2_cl = PI2*G_m/(1+PI2*G_m);% model llaç tancat amb PI, fórmula de Mason
    p2_cl = pole(G2_cl);
    
    [omg_,T,X] = lsim(G2_cl,u_,t_);  
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
title(['Resposta control PI, consigna ' num2str(u) ' eu'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('Resposta [eu]','Interpreter','latex','FontSize',24);
legend(legend_,'Interpreter','latex','FontSize',18,'Location','south');
set(gca,'FontSize',18); legend boxoff; grid;
saveas(H2_cl,'./figures/Exemple_2_PI_1r_ordre_2cl.pdf')
saveas(H2_cl,'./figures/Exemple_2_PI_1r_ordre_2cl.svg')

%% APARTAT: Error de variació de sortida: 'Error de velocitat'

e_v=1/(1/Ti*K); % eu, error en la derivada de del valor de sortida

disp(["L'error estacionari és de "+num2str(round(e_v,4))+' eu'])
