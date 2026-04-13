%%% Obtenir l'eqüació 2 gdl en espai d'estat
% Obtenir el diagrama de Bode i la resposta transitòria
% J.A.Mayugo, UdG, 2014,2023

clear; close all;

%% Propietats del sistema
% Motor	
n_max = 3600;   % rpm,      velocitat nominal
Mm_n  = 3.53;   % Nm,       parell nominal
Mm_max= 12.2;   % Nm,       parell màxim
J_m   = 260e-6; % kg m2,    inercia motor
b_m   = 1e-3;   % Nms/rad,  pèrdues motor
% Cargol potència
pas = 10e-3;    % m,        pas (relació de transmissió de 10 mm/rev) 
D   = 23.2e-3;  % m,        diàmetre mig cargol
L   = 940e-3;   % m,        longitud del cargol
rho = 7.85e3;   % kg/m3,    densitat acer cargol
% Acoplament
J_a = 30e-6;    % kg m2,    inercia acoplament
k_a = 120e-3;   % Nm/rad, rigidesa acoplament
% Massa carro
m_carro = 1.35; % kg, massa carro
M       = 0;    % kg, massa càrrega

n_gdl =2    % Sistema mecànic 2gdl

%% RESOLUCIÓ 
% Propietats cargol
v_cargol = pi*D^2/4*L           % m3, volum del cargol
m_cargol = v_cargol*rho         % kg, massa del cargol
J_cargol = 1/2*m_cargol*(D/2)^2 % kg m2, inercia cargol
i_n = pas/(2*pi)                % m/rad, relacio transmissio cargol

% Propietats físiques motor cc
w_m_n = 3600/30*pi * (12.2-3.53)/12.2  % velocitat nominal [rad/s]
M_m_n = 3.53                % moment nominal [Nm]
V_m_n = 120                 % voltatge nominal [V]
A_m_n = (w_m_n*M_m_n)/V_m_n % intensitat nominal [A]

K_m = M_m_n/A_m_n   % Nm / A
K_b = V_m_n/w_m_n   % V  / rad/s
L = 0.82e-3         % mH
R = 1.6             % ohms
b_m = 1e-3          % Nms/rad

J = J_m+J_a+J_cargol+(m_carro+M)*i_n^2;
b = b_m; % K_m*K_b/R + b_m;
K = K_m; % K_m=K_b

%% Model
s = tf('s');
model_M = K/((J*s+b)*(L*s+R)+K^2);
% model_M = K/((J*s+b)*(R)+K^2);

%% Resposta freqüencial (H(W))
[omega_n]   = abs(pole(model_M))          % Freqüència natual
[zeta]      = abs(real(pole(model_M)))./abs(pole(model_M))% Factor esmorteïment
[Wn,zeta_c] = damp(model_M)               % Factor esmorteïment (alternative)
[omega_r]   = abs(imag(pole(model_M)))    % Freqüència resonància

H1=figure;
opts = bodeoptions('cstprefs');
%opts.FreqUnits = 'Hz';
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode log-log';
% h1 = bodeplot(model_M,model0(1,n_gdl),{1,max(omega_r)*10}); % Diagrama de Bode
h1 = bodeplot(model_M,{1,max(omega_n)*10},opts); % Diagrama de Bode
%saveas(H1,'./figures/CSM_cargol_01_din_1b_1.svg')
%saveas(H1,'./figures/CSM_cargol_01_din_1b_1.pdf')

H2=figure;
opts = bodeoptions('cstprefs');
%opts.FreqUnits = 'Hz';
opts.FreqScale = 'linear';
opts.MagUnits = 'abs';
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode lineal-lineal';
h2 = bodeplot(model_M,{1,max(omega_n)*1.5},opts); % Diagrama de Bode
%setoptions(h2,'FreqScale','linear')
%setoptions(h2,'MagUnits','abs')
%%setoptions(h2,'FreqUnits','Hz')
%saveas(H2,'./figures/CSM_cargol_01_din_1b_2.svg')
%saveas(H2,'./figures/CSM_cargol_01_din_1b_2.pdf')

%% Resposta transitòria
% escaló en llaç obert:
u = 120;  %Volts al motor 
H3=figure;
subplot(1,2,1);
step(u * model_M)
grid; xlabel('temps (s)'); ylabel('velocitat (rad/s)')
subplot(1,2,2);
step(u * model_M)
grid; xlabel('temps (s)'); ylabel('velocitat (m/s)')
%saveas(H3,'./figures/CSM_cargol_01_din_1b_3.svg')
%saveas(H3,'./figures/CSM_cargol_01_din_1b_3.pdf')

%% Resposta temporal impuls extern (canvi voltatge)
t_f = 0.06
t  = 0:1e-5:t_f;
V_ = [u*(rectpuls(t,t_f))];
[out,T,X] = lsim(model_M,V_,t);  % resposta model a canvi voltatge

H4=figure;
H4.Position = [10 10 560 700];
subplot(3,1,1);
hold on
plot(t,V_(1,:),'LineWidth',1.2)             % input Voltatge motor
ylabel('Voltatge (V)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,2);
hold on
plot(t,out*30/pi,'-r',    'LineWidth',1.2)  % resposta gdl rpm 
legend({'eix motor'},'FontSize',18,'Location','Best')
ylabel('velocitat (rpm)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,3);
plot(t,out*i_n,'LineWidth',1.2,'Color',[0.466 0.674 0.188]) % resposta carro
legend('carro','FontSize',18,'Location','Best')
xlabel('temps (s)','FontSize',18);ylabel('velocitat (m/s)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
%saveas(H4,'./figures/CSM_cargol_01_din_1b_4.svg')
%saveas(H4,'./figures/CSM_cargol_01_din_1b_4.pdf')

%% Resposta temporal impuls extern (canvi voltatge PWM 80% de 120 v.)
t_f = 0.06
t   = 0:1e-5:t_f;

Npulses =100;  % #, nombre de polsos 
Duty    = 80;  % %, de 0 a 100%,  percentatge del cicle de treball

V_ = u*rectpuls(t,t_f).*...
     pulstran(t,[0:1/Npulses:1]*t_f,@rectpuls,Duty/100*t_f/Npulses);
[out,T,X] = lsim(model_M,V_,t);  % resposta model a canvi voltatge

H5=figure;
H5.Position = [20 20 560 700];
subplot(3,1,1);
hold on
plot(t,V_(1,:),'LineWidth',1.2)        % input Voltatge motor
ylabel('Voltatge (V)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,2);
hold on
plot(t,out*30/pi,'-r','LineWidth',1.2) % resposta a rpm 
legend({'eix motor'},'FontSize',18,'Location','Best')
ylabel('velocitat (rpm)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,3);
plot(t,out*i_n,'LineWidth',1.2,'Color',[0.466 0.674 0.188]) % resposta carro
legend('carro','FontSize',18,'Location','Best')
xlabel('temps (s)','FontSize',18);ylabel('velocitat (m/s)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
%saveas(H4,'./figures/CSM_cargol_01_din_1b_4.svg')
%saveas(H4,'./figures/CSM_cargol_01_din_1b_4.pdf')