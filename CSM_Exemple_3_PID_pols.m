%%% Exemple sintonització PID per control de pols sistema
% Implementar PID
% J.A.Mayugo, UdG, 2023

clear; close all;

%% Propietats del sistema
s = tf('s');
G_m = (s+1)/(s+3);

err_v = 0.5;

t_ = 0:0.001:7;    % resposta temporal

%% Resposta en llaç obert:
U  = 1;         % eu, input 'bump test'
U_ = U *ones(size(t_));
[omg,T,X]         = lsim(G_m,U_,t_);  % voltage/angular vel.
H_ol=figure;
plot(t_,omg,'LineWidth',1.8)
grid; title(['Resposta a una funció escaló amb ' num2str(U) ' ue'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('Resposta [ue]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18)
%saveas(H_ol,'./figures/Exemple_PI_1r_ordre_ol.pdf')


%% PID implementació i sintonització
Ki = 3/err_v;
Kp = 94/65;
Kd = 43/65;

PID = Kp+Ki/s+Kd*s                 % controlador PID
G_cl = minreal(PID*G_m/(1+PID*G_m))% model llaç tancat, fórmula de Mason
p_cl = pole(G_cl)

u = 1;   % eu, input consigna
u_ = u *ones(size(t_));
[pos_,T,X]         = lsim(G_cl,u_,t_);    % input/posició
[vel_,T,X]         = lsim(G_cl/s,u_,t_);  % input/velocitat

H_cl=figure;
tl = tiledlayout('flow','TileSpacing','compact');
nexttile;hold on;
plot(t_,u_,'LineWidth',1.2)
plot(t_,pos_,'LineWidth',1.8)
ylabel('consigna [eu]','Interpreter','latex','FontSize',24)
ylim([0 1.2])
set(gca,'FontSize',18);grid; 
nexttile;hold on;
plot(t_,t_,'LineWidth',1.2)
plot(t_,vel_,'LineWidth',1.8)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('velocitat [m/s]','Interpreter','latex','FontSize',24)
set(gca,'FontSize',18);grid; 

lgd = legend({'consigna','resposta'},'Location','northoutside' ...
        ,'Interpreter','latex','FontSize',20,'NumColumns',2);
legend box off;
lgd.Layout.Tile = 'south';

saveas(H_cl,'./figures/Exemple_3_PID_pols.pdf')


