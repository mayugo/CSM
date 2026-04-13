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

%% Matrius de massa M_, esmorteïment C_ i rigidesa K_
J_e = J_m +  J_a + J_cargol  + (m_carro + M)*(i_n*i_n)

J11 = J_m + J_a/2;
J22 = (J_a/2+J_cargol)/i_n^2 + m_carro + M; % si segon gdl moviment carro
%J22 = J_a/2+J_cargol + (m_carro+M)*i_n^2;  % si segon gdl eix cargol

C11 = b_m;

K11 = k_a;
K12 = -k_a/i_n;
K22 = k_a/i_n^2;                            % si segon gdl moviment carro
% K12 = -k_a; K22 = k_a;                    % si segon gdl eix cargol

M_ = [J11   0  ;   0   J22]
C_ = [C11   0  ;   0    0 ]
K_ = [K11  K12 ;  K12  K22]

%% Definició de l'espai d'estat

A = [ zeros(n_gdl)    eye(n_gdl);
      -inv(M_)*K_    -inv(M_)*C_]
A0 = [ zeros(n_gdl)   eye(n_gdl);
      -inv(M_)*K_    -inv(M_)*zeros(n_gdl);]  % Matriu A sense esmorteïment

B = [ zeros(n_gdl) ; inv(M_)];      % input voltatge motor

C = [ zeros(n_gdl)  eye(n_gdl) ];   % defineix output velocitats
%C = [ eye(n_gdl)  zeros(n_gdl) ];  % defineix output posicions

D = [ zeros(n_gdl) ];

model  = ss(A, B, C, D);
model0 = ss(A0, B, C, D);

%% Resposta freqüencial (H(W))
[omega_n]   = abs(pole(model))          % Freqüència natual
[zeta]      = abs(real(pole(model)))./abs(pole(model))% Factor esmorteïment
[Wn,zeta_c] = damp(model)               % Factor esmorteïment (alternative)
[omega_r]   = abs(imag(pole(model)))    % Freqüència resonància

H1=figure;
opts = bodeoptions('cstprefs');
%opts.FreqUnits = 'Hz';
opts.XLabel.FontSize = 18;
opts.YLabel.FontSize = 18;
opts.TickLabel.FontSize = 16;
opts.Title.FontSize = 18;
opts.Title.String = 'Diagrama de Bode log-log';
% h1 = bodeplot(model(1,n_gdl),model0(1,n_gdl),{1,max(omega_r)*10}); % Diagrama de Bode
h1 = bodeplot(model(1,n_gdl),{1,max(omega_r)*10},opts); % Diagrama de Bode
%saveas(H1,'./figures/CSM_cargol_01_din_1a_1.svg')
%saveas(H1,'./figures/CSM_cargol_01_din_1a_1.pdf')

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
h2 = bodeplot(model(1,n_gdl),{1,max(omega_r)*1.5},opts); % Diagrama de Bode
%setoptions(h2,'FreqScale','linear')
%setoptions(h2,'MagUnits','abs')
%%setoptions(h2,'FreqUnits','Hz')
%saveas(H2,'./figures/CSM_cargol_01_din_1a_2.svg')
%saveas(H2,'./figures/CSM_cargol_01_din_1a_2.pdf')

%% Resposta transitòria
% escaló en llaç obert:
M0 = Mm_n/10;  %M Moment motor (10% nominal)
H3=figure;
subplot(1,2,1);
step(M0 * model(1,1))
grid; xlabel('temps (s)'); ylabel('velocitat (rad/s)')
subplot(1,2,2);
step(M0 * model(1,2))
grid; xlabel('temps (s)'); ylabel('velocitat (m/s)')
%saveas(H3,'./figures/CSM_cargol_01_din_1a_3.svg')
%saveas(H3,'./figures/CSM_cargol_01_din_1a_3.pdf')

%% Resposta temporal impuls extern (canvi moment motor)
t_final = 6
t  = 0:1e-5:t_final;
M0_ = [M0*(rectpuls(t,t_final));0*t];

[out,T,X] = lsim(model,M0_,t);  % resposta model a canvi moment M0

H4=figure;
H4.Position = [10 10 560 700];
subplot(3,1,1);
hold on
plot(t,M0_(1,:),'LineWidth',1.2) % input Moment motor
ylabel('Parell motor (Nm)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,2);
hold on
plot(t,out(:,1)*30/pi,'-r',    'LineWidth',1.2) % resposta gdl 1 a rpm 
plot(t,out(:,2)*30/pi/i_n,'-b','LineWidth',1.2) % resposta gdl 2 a rpm
legend({'eix motor','eix cargol'},'FontSize',18,'Location','Best')
ylabel('velocitat (rpm)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
subplot(3,1,3);
plot(t,out(:,2),'LineWidth',1.2,'Color',[0.466 0.674 0.188]) % resposta carro
legend('carro','FontSize',18,'Location','Best')
xlabel('temps (s)','FontSize',18);ylabel('velocitat (m/s)','FontSize',18)
set(gca,'FontSize',18),hold off,grid on
%saveas(H4,'./figures/CSM_cargol_01_din_1a_4.svg')
%saveas(H4,'./figures/CSM_cargol_01_din_1a_4.pdf')