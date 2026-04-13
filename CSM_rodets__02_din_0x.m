%% Sol·lució Exercici transportador rodets caixes
% Implemntacio model dinàmic
% J.A.Mayugo, UdG, 2021

close all; clear
%% DADES: propietats del sistema
% -- dades motor MAXON, 20W, A-max 2019
n_max   = 6460;     % rpm,      velocitat màxima
n_nom   = 5060;     % rpm,      velocitat nominal
Pot_nom = 120;      % Watts,    potència nominal
V_nom   = 24;       % Volts,    voltatge nominal
R       = 3.99;     % ohms,     resitència induït
L       = 0.556e-3; % H,        impedància induït
mmot    = 0.240;    % kg,       massa motor
Jmot    = 45.3e-3*1e-4;% kg·m2, inècia motor (45.3grcm2)
bm      = 2.8e-6;   % Ns/m,     dissipació motor
% -- dades del reductor 
i       = 168;      %   ,       relació de transmissió reductor    
mred    = 0.19;     % kg,       massa del reductor
Jred    = 0.7e-7*i^2;% kg·m2,   inèrcia reductor eix lent (0.7grcm2 eix rap)
% -- dimensions, masses i inèrcie elements màquina
rR      =  50e-3;   % m,        radi rodet
rP      =  40e-3;   % m,        radi politja cadena
pitch   = 125e-3;   % m,        distància entre rodets
mR      = 6.5;      % kg,       massa rodet
JR      = 6.2*(25e-3)^2;%kg m2, inèrcia de masses rodet
cR       = 0.28e-3;  % Ns/m,     esmorteiment eix cada rodet
NR      = 11;       % #,        nombre de rodets
mC      = 0.360;    % kg,       massa caixa
NC      = 5;        % #,        nombre de caixes

%% Resolució APARTAT a)
i_t     = i/rR;
omg_nom = n_nom*pi/30;
vC      = rR * omg_nom/i

%% Resolució APARTAT b)
omg_n   = n_nom/30*pi           % rad/s
M_nom   = Pot_nom/omg_n         % Nm
A_nom   = (omg_n*M_nom)/V_nom   % A
K_m     = M_nom/A_nom           % Nm / A
K_b     = V_nom/omg_n           % V s / rad
P_nom   = omg_n*M_nom           % W

%% Resolució APARTAT c)
J_e     = Jmot + Jred/i^2 + JR*NR/i^2 + mC*NC/i_t^2
b_e     = bm + cR/i^2 * NR      % esmorteïment sistema mecànic
b_e_    = b_e + (K_m*K_b)/R     % esmorteïment mecànic més elèctric

% Model tenint en compte el factor s^2 del denominador (J_e*L)
s = tf('s');
model_omg_m  = K_m/((J_e*s+b_e)*(L*s+R)+K_b*K_m)
model_u_c    = model_omg_m/s/i_t  % sortida desplaçament de les caixes

% Model simplificat a odre 1
model_omg_mS = K_m/((J_e*s+b_e)*(R)+K_b*K_m) % model simplicat
model_u_cS   = model_omg_mS/s/i_t % model simplicat

%% Resposta freqüencial (H(W))
% Diagrama de Bode i R-Locus
H01=figure;
opts = bodeoptions('cstprefs');
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode log-log';
h1 = bodeplot(model_omg_m,model_u_c,opts); %,{0.1,20});%setoptions(h1,'MagUnits','abs','FreqScale','linear')
set(H01,'Units','Inches');pos = get(H01,'Position');
set(H01,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(H01,'./figures/CSM_rodets_02_din_0x_1','-dsvg','-r0')
print(H01,'./figures/CSM_rodets_02_din_0x_1','-dpdf','-r0','-bestfit')

%% Resposta transitòria
% escaló en llaç obert:
u = V_nom;  %Volts al motor 
H02=figure;
subplot(2,1,1);
step(u * model_omg_m,0.06)
grid;title(''),set(gca,'FontSize',16)
xlabel('temps (s)','FontSize',16),ylabel('velocitat motor (rad/s)','FontSize',16)
subplot(2,1,2);
step(u * model_u_c,0.06)
grid;title(''),set(gca,'FontSize',16)
xlabel('temps (s)','FontSize',16);ylabel('desplaçament carro (m)','FontSize',16)
set(H02,'Units','Inches');pos = get(H02,'Position');
set(H02,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(H02,'./figures/CSM_rodets_02_din_0x_2','-dsvg','-r0')
print(H02,'./figures/CSM_rodets_02_din_0x_2','-dpdf','-r0','-bestfit')

%% Resposta temporal (canvi voltatge PWM 90, 60 i 30% de 120 v.)
t_f = 0.06
t   = 0:1e-5:t_f;
V_ = u*ones(1,length(t));
[out_m,T,X] = lsim(model_omg_m,V_,t);  % resposta model a canvi voltatge
[out_u,T,X] = lsim(model_u_c,V_,t);  % resposta model a canvi voltatge

Npulses = 12;  % #, nombre de polsos 
Duty    = 90;  % %, de 0 a 100%,  percentatge del cicle de treball
V9_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out9_m,T,X] = lsim(model_omg_m,V9_,t);  % resposta model a canvi voltatge
[out9_u,T,X] = lsim(model_u_c,V9_,t);  % resposta model a canvi voltatge

Duty    = 60;  % %, de 0 a 100%,  percentatge del cicle de treball
V6_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out6_m,T,X] = lsim(model_omg_m,V6_,t);  % resposta model a canvi voltatge
[out6_u,T,X] = lsim(model_u_c,V6_,t);  % resposta model a canvi voltatge

Duty    = 30;  % %, de 0 a 100%,  percentatge del cicle de treball
V3_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out3_m,T,X] = lsim(model_omg_m,V3_,t);  % resposta model a canvi voltatge
[out3_u,T,X] = lsim(model_u_c,V3_,t);  % resposta model a canvi voltatge

H04=figure;
H04.Position = [20 20 900 700];
subplot(3,3,1);
plot(t,V9_(1,:),'LineWidth',1.2,'Color',[0 0.45 0.74])        % input Voltatge motor
leg1=legend({'PWM 90%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
ylabel('voltatge (V)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,4);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out9_m*30/pi,'LineWidth',1.2,'Color',[0 0.45 0.74]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 90%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
ylabel('velocitat (rpm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,7);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out9_u*1e3,'LineWidth',1.2,'Color',[0 0.45 0.74]) % resposta carro
leg3=legend({'carro a V_n','carro a 90%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16);ylabel('desplaçament (mm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])

subplot(3,3,2);
plot(t,V6_(1,:),'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % input voltatge motor
leg1=legend({'PWM 60%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,5);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out6_m*30/pi,'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 60%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,8);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out6_u*1e3,'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % resposta carro
leg3=legend({'carro a V_n','carro a 60%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16);ylabel('desplaçament (mm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])

subplot(3,3,3);
plot(t,V3_(1,:),'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % input voltatge motor
leg1=legend({'PWM 30%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,6);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out3_m*30/pi,'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 30%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,9);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out3_u*1e3,'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % resposta carro
leg3=legend({'carro a V_n','carro a 30%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16);ylabel('desplaçament (mm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])
print(H04,'./figures/CSM_rodets_02_din_0x_4','-dsvg','-r0')
print(H04,'./figures/CSM_rodets_02_din_0x_4','-dpdf','-r0','-bestfit')

Npulses = 60;  % #, nombre de polsos 
Duty    = 90;  % %, de 0 a 100%,  percentatge del cicle de treball
V9_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out9_m,T,X] = lsim(model_omg_m,V9_,t);  % resposta model a canvi voltatge
[out9_u,T,X] = lsim(model_u_c,V9_,t);  % resposta model a canvi voltatge

Duty    = 60;  % %, de 0 a 100%,  percentatge del cicle de treball
V6_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out6_m,T,X] = lsim(model_omg_m,V6_,t);  % resposta model a canvi voltatge
[out6_u,T,X] = lsim(model_u_c,V6_,t);  % resposta model a canvi voltatge

Duty    = 30;  % %, de 0 a 100%,  percentatge del cicle de treball
V3_ = u*pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out3_m,T,X] = lsim(model_omg_m,V3_,t);  % resposta model a canvi voltatge
[out3_u,T,X] = lsim(model_u_c,V3_,t);  % resposta model a canvi voltatge

H05=figure;
H05.Position = [20 20 900 700];
subplot(3,3,1);
plot(t,V9_(1,:),'LineWidth',1.2,'Color',[0 0.45 0.74])        % input Voltatge motor
leg1=legend({'PWM 90%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
ylabel('voltatge (V)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,4);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out9_m*30/pi,'LineWidth',1.2,'Color',[0 0.45 0.74]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 90%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
ylabel('velocitat (rpm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,7);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out9_u*1e3,'LineWidth',1.2,'Color',[0 0.45 0.74]) % resposta carro
leg3=legend({'carro a V_n','carro a 90%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16);ylabel('desplaçament (mm)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])

subplot(3,3,2);
plot(t,V6_(1,:),'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % input voltatge motor
leg1=legend({'PWM 60%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,5);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out6_m*30/pi,'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 60%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,8);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out6_u*1e3,'LineWidth',1.2,'Color',[0.85 0.32 0.1]) % resposta carro
leg3=legend({'carro a V_n','carro a 60%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])

subplot(3,3,3);
plot(t,V3_(1,:),'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % input voltatge motor
leg1=legend({'PWM 30%'},'FontSize',16,'Location','North'),set(leg1,'Box','off')
set(gca,'FontSize',16),hold off,grid on,ylim([0 40])
subplot(3,3,6);
hold on
plot(t, out_m*30/pi,'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out3_m*30/pi,'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % resposta a rpm 
leg2=legend({'eix a V_n','eix a 30%'},'FontSize',16,'Location','North'),set(leg2,'Box','off')
set(gca,'FontSize',16),hold off,grid on, ylim([0 8000])
subplot(3,3,9);
hold on
plot(t,out_u*1e3, 'LineWidth',1.0,'Color',[.7 .7 .7]) % resposta a rpm 
plot(t,out3_u*1e3,'LineWidth',1.2,'Color',[0.47 0.67 0.19]) % resposta carro
leg3=legend({'carro a V_n','carro a 30%'},'FontSize',16,'Location','North'),set(leg3,'Box','off')
xlabel('temps (s)','FontSize',16)
set(gca,'FontSize',16),hold off,grid on, ylim([0 15])
print(H05,'./figures/CSM_rodets_02_din_0x_5','-dsvg','-r0')
print(H05,'./figures/CSM_rodets_02_din_0x_5','-dpdf','-r0','-bestfit')

%% Resolució APARTAT d)
% Matrius de massa M_, esmorteïment C_ i rigidesa K_
f_v  = [K_m/R]     % adaptar inputs en voltatge
M_ = [J_e]   
C_ = [b_e_]
K_ = [0]

% Definició de l'espai d'estat
n_gdl = size(M_,1)
A = [ zeros(n_gdl) eye(n_gdl);  -inv(M_)*K_    -inv(M_)*C_]
B = [ zeros(n_gdl) ; inv(M_)]*f_v       % inputs voltatges motors
C = [ 0  1 ; 1/i_t  0];       % defineix output velocitats angulars
D = [ 0 ; 0];
model = ss(A, B, C, D,  'statename', {'\theta_{motor}' 'omg_motor'},...
                        'inputname', {'U_{motor} (V)'},...
                        'outputname',{'omg_{motor} (rad/s)' 'desp_{caixa} (m/s)'})

% Diagrama de Bode i R-Locus
H11=figure;
opts = bodeoptions('cstprefs');
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode log-log';
h1 = bodeplot(model,opts); %,{0.1,20});%setoptions(h,'MagUnits','abs','FreqScale','linear')
set(H11,'Units','Inches');pos = get(H11,'Position');
set(H11,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(H11,'./figures/CSM_rodets_02_din_0x_11','-dsvg','-r0')
print(H11,'./figures/CSM_rodets_02_din_0x_11','-dpdf','-r0','-bestfit')


%% Resposta transitòria
% escaló en llaç obert:
u = V_nom;  %Volts al motor 
H12=figure;
step(u * model,0.06)
grid;title(''),%set(gca,'FontSize',16)
xlabel('temps (s)','FontSize',16),ylabel('desp. carro (m);  velocitat motor (rad/s)','FontSize',16)
set(H12,'Units','Inches');pos = get(H12,'Position');
set(H12,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(H12,'./figures/CSM_rodets_02_din_0x_12','-dsvg','-r0')
print(H12,'./figures/CSM_rodets_02_din_0x_12','-dpdf','-r0','-bestfit')


%% Conversió d'espai d'estat H_s = C*inv(s*I-A)*B+D
[num,den] = ss2tf(A,B,C,D,1)  
H_s11 = tf(num(1,:),den);H_s11=minreal(H_s11)

%FT1= H_s11*R
%fig1=figure;bode(H_s11,model_omg_mS);%legend('H_{s11}','H_{s22}')
%fig2=figure;rlocus(H_s11);%title('Root Locus H_{s11}')