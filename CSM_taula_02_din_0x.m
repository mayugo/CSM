%% Sol·lució implementada sistema MISO
% Generació de trajectòries
% J.A.Mayugo, UdG, 2023

close all, clear

%% DADES
% dades motor 
n_nom 	= 1470  % rpm,      velocitat nominal motor
Pot_nom = 1600  % W,        potència nominal motor
mmot    = 6     % kg,       massa motor
Jmot    = 0.01  % kg·m2,    inèrcia motor
% dades del reductor 40:1
i       = 40    % adim,     relació de reducció reductor
mred    = 54    % kg,       massa del reductor
Jred    = 1.2   % kg·m2,    inèrcia del cos del reductor
Jsor    = 2     % kg·m2,    inèrcia eixos i rodes reductor
% peces
Jtau    = 10    % kg·m2,    inèrcia taula
Npec    = 4     % #,        nombre de peces 
mpec    = 12    % kg,       massa de cada peça
R       = 0.5   % m,        radi peces/taula
% Força fregament, paràmetres
F1 = 100
F2 = 50     % on F_frec = F1 + F2*omega_tau

%% Resolució

Jeq = Jmot +(Jsor + Jtau)/(i*i) + Npec*mpec*R*R/(i*i)

beq = F2*R/i^2

M_r_eq = F1*R/i

M_m = Pot_nom/(n_nom*pi/30)


%% Model Dinàmic
s = tf('s');          

Numerator   = {  [1]         [-1]};      %Numerators of u_1 and u_2
Denominator_G = {[Jeq beq]   [Jeq beq]}; %Denominators of u_1 and u_2
Denominator_H = {[Jeq beq 0] [Jeq beq 0]}; %Denominators of u_1 and u_2
G = tf(Numerator,Denominator_G);
H = tf(Numerator,Denominator_H);

%% Resposta freqüencial (H(W))
% Diagrama de Bode i R-Locus
H01=figure;
opts = bodeoptions('cstprefs');
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode log-log';
h1 = bodeplot(G,opts); %,{0.1,20});%setoptions(h1,'MagUnits','abs','FreqScale','linear')
set(H01,'Units','Inches');pos = get(H01,'Position');
set(H01,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(H01,'./figures/CSM_rodets_02_din_0x_1','-dsvg','-r0')
% print(H01,'./figures/CSM_rodets_02_din_0x_1','-dpdf','-r0','-bestfit')

%% Resposta transitòria
% escaló en llaç obert:
t_f = 20;

t   = 0:1e-3:t_f;
M_m_    = M_m*ones(1,length(t));
M_r_eq_ = M_r_eq*ones(1,length(t));
[outG_1,T,X] = lsim(G(1)*exp(-10*s),M_m_,t);  % resposta model velocitat
[outG_2,T,X] = lsim(G(2)*exp(0*s),M_r_eq_,t);  % resposta model velocitat
[outH_1,T,X] = lsim(H(1)*exp(-10*s),M_m_,t);  % resposta model gir
[outH_2,T,X] = lsim(H(2)*exp(0*s),M_r_eq_,t);  % resposta model gir

[outG_T,T,X] = lsim(G(1)*exp(-10*s),M_m_-M_r_eq_,t);  % resposta model velocitat
[outH_T,T,X] = lsim(H(1)*exp(-10*s),M_m_-M_r_eq_,t);  % resposta model velocitat


H02=figure;
H02.Position = [20 20 900 700];
subplot(3,2,1);
plot(t,outG_1,'LineWidth',1.2,'Color',[0 0.45 0.74])        % output 1
leg1=legend({'Resposta al motor motor'},'FontSize',16,'Location','best'),set(leg1,'Box','off')
ylabel('velocitat [rad/s]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,%ylim([0 40])
subplot(3,2,2);
plot(t,outH_1,'LineWidth',1.2,'Color',[0 0.45 0.74])        % output 1
leg1=legend({'Resposta al motor motor'},'FontSize',16,'Location','best'),set(leg1,'Box','off')
ylabel('gir [rad]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,%ylim([0 40])
subplot(3,2,3);
plot(t,outG_2,'LineWidth',1.2,'Color',[0 0.74 0.45])        % output 2
leg1=legend({'Resposta a la força fregament'},'FontSize',16,'Location','best'),set(leg1,'Box','off')
ylabel('velocitat [rad/s]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,%ylim([-100 0])
subplot(3,2,4);
plot(t,outH_2,'LineWidth',1.2,'Color',[0 0.74 0.45])        % output 2
leg1=legend({'Resposta a la força fregament'},'FontSize',16,'Location','best'),set(leg1,'Box','off')
ylabel('gir [rad]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,%ylim([-1000 0])
subplot(3,2,5);
hold on;
plot(t,max(0,outG_1+outG_2)-0,'LineWidth',1.2,'Color',[0 0 0.65])        
plot(t,outG_T,'LineWidth',1.2,'Color',[0.9 0.5 0.65])       
leg1=legend({'Resposta global (superposició)','Resposta global SISO'},'FontSize',16,'Location','north'),set(leg1,'Box','off')
ylabel('velocitat [rad/s]','FontSize',16)
xlabel('temps [s]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,ylim([0 800])
subplot(3,2,6);hold on;
plot(t,max(min(outH_1+outH_2),outH_1+outH_2)-min(outH_1+outH_2),'LineWidth',1.2,'Color',[0 0 0.65])        
plot(t,outH_T,'LineWidth',1.2,'Color',[0.9 0.5 0.65])        
leg1=legend({'Resposta global (superposició)','Resposta global SISO'},'FontSize',16,'Location','best'),set(leg1,'Box','off')
ylabel('gir [rad]','FontSize',16)
xlabel('temps [s]','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,%ylim([0 800])
set(H02, 'Renderer', 'painters');

print(H02,'./figures/CSM_taula_02_din_0x_2','-dsvg','-r0','-painters')
print(H02,'./figures/CSM_taula_02_din_0x_2','-dpdf','-r0','-bestfit')