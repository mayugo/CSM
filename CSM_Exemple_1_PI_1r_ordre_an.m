%%% Exemple sintonització analítica PI de 1r ordre
% Identificar el model, el diagrama de Bode, i implementar PI
% J.A.Mayugo, UdG, 2022

clear; close all;

%% Propietats del sistema
U  = 1;         % eu, input 'bump test'

t_ = 0:0.01:0.5*10;    % resposta temporal

s = tf('s');
G_m = 1/(s-1);

% Resposta en llaç obert:
U_ = U *ones(size(t_));
[omg,T,X]         = lsim(G_m,U_,t_);  % voltage/angular vel.
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló amb ' num2str(U) ' ue'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('Resposta [ue]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
%saveas(H_ol,'./figures/Exemple_PI_1r_ordre_ol.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab i R-Locus
H_bode=figure
h = bodeplot(G_m); %,{0.1,20});
%setoptions(h,'MagUnits','abs','FreqScale','linear')

H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

%% APARTAT: Condició d'estabilitat
Kp_ = [ 10 10 10 10 ];%*1/5;
Ki_ = [ 3  6 12 24 ];%*1/5;
u = 1;
u_ = u *ones(size(t_));
for ii = 1:length(Kp_)  
    PI   = Kp_(ii)+ (Ki_(ii)/s); % controlador PI
    G_cl = PI*G_m/(1+PI*G_m);% model llaç tancat amb PI, fórmula de Mason
    p_cl = pole(G_cl);
    
    [omg_,T,X] = lsim(G_cl,u_,t_);  % resposta
    OMG_(ii,:)=omg_;
end

H2_cl=figure;
hold on
legend_=string(Kp_);
for ii = 1:length(Kp_)
    plot(t_,OMG_(ii,:),'LineWidth',1.8);
    legend_(ii) = "$K_p$ = " + num2str(round(Kp_(ii),1)) + ...
                  ", $K_i$ = " + num2str(round(Ki_(ii),1));
end
grid; title(['Resposta control PI, consigna ' num2str(u) ' eu'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('Resposta [eu]','Interpreter','latex','FontSize',24);
legend(legend_,'Interpreter','latex','FontSize',18,'Location','south');
set(gca,'FontSize',18);legend boxoff
saveas(H2_cl,'./figures/Exemple_PI_1r_A.pdf')