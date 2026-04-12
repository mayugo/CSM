%% Sol·lució PAC 2021
% Generació de trajectòries
% J.A.Mayugo, UdG, 2021

close all, clear
%% DADES
% -- dades del reductor 
i = 168  
% --  dimensions
rR = 50e-3
rP =  40e-3
pitch = 130e-3
rE = 400e-3
% -- 
NR = 11
NC = 1

% dades APARTAT a)
v_maxE  = 0.15  % m/s
tE      = 2.0   % segons
t_bE    = 0.5   % segons

% dades APARTAT b)
a_max   = 1.5;  % m/s2, acceleració màxima 
tt      = 5;  % segons, temps total moviment
lAB     = (NR-1)*pitch;  %-- metres
x       = lAB;    % m, recorregut total 

i_t = i/rR;

%% Resolució APARTAT a), marxa-moviment-parada
ttE = tE + 2*t_bE;         % temps total 
a_maxE = v_maxE/t_bE;      % accceleració màxima
x_bE = 1/2*a_maxE*t_bE^2;  % desplaçament en zona de t_b

% en el rodet
omg_maxE = v_maxE/rE
eps_maxE = a_maxE/rE

% Definir trams de moviment: marxa-moviment-parada
tx_a(1) = t_bE;
tx_a(2) = tx_a(1) + ttE-2*t_bE;
tx_a(3) = tx_a(2) + t_bE;

x_a(1) =        + 1/2*a_maxE*tx_a(1)^2;
x_a(2) = x_a(1) + v_maxE*(tx_a(2)-tx_a(1));
x_a(3) = x_a(2) + v_maxE*(tx_a(3)-tx_a(2)) - 1/2*a_maxE*(tx_a(3)-tx_a(2))^2;
xxE = x_a(3);
xE = x_a(2)-x_a(1);

% Obtenir la trajectoria posicio, velocitat i acceleració
t_ = [0:0.01:ttE];
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if      t_(ii) <= tx_a(1)
        x_(ii) = + 1/2*a_maxE*t_(ii)^2;
        v_(ii) = a_maxE*t_(ii);
        a_(ii) = a_maxE;
    elseif  t_(ii) <= tx_a(2)
        x_(ii) = x_a(1) + v_maxE*(t_(ii)-tx_a(1));
        v_(ii) = v_maxE;
        a_(ii) = 0;
    else %if  t_(ii) <= t_i(3)
        x_(ii) = x_a(2) + v_maxE*(t_(ii)-tx_a(2)) - 1/2*a_maxE*(t_(ii)-tx_a(2))^2;
        v_(ii) = v_maxE - a_maxE*(t_(ii)-tx_a(2));
        a_(ii) = -a_maxE;
    end
end

%  Tram & tipus & $t$ [s] & $u$ [m] & $\dot{u}$ [m/s] & $\ddot{u}$ [m/s\tss{2}]]    \\
disp(['Tram 1 , 0 a 1, acceleració',', t_1 =',num2str(t_bE,'%10.3f'),'s , u_1 =',num2str(x_a(1),'%10.3f'),'m , a_max =',num2str(a_maxE,'%10.3f') 'm/s2' ])
disp(['Tram 2 , 1 a 2, vel. constant',', t_2 =',num2str(tE-2*t_bE,'%10.3f'),'s , u_2 =',num2str(x_a(2)-x_a(1),'%10.3f'),'m , v_max =',num2str(v_maxE,'%10.3f') 'm/s' ])
disp(['Tram 3 , 2 a 3, desacceleració',', t_3 =',num2str(t_bE,'%10.3f'),'s , u_3 =',num2str(x_a(3)-x_a(2),'%10.3f'),'m , a_max =',num2str(-a_maxE,'%10.3f') 'm/s2' ])
disp(' ')
disp(['temps total =',num2str(ttE,'%10.2f'),'s , xE total =',num2str(abs(xxE),'%10.2f'), 'm'])

% Representar posició, velocitat i acceleració
H1=figure;font_size=18;
subplot(3,1,1);
plot(t_,x_,'LineWidth',2)
ylabel('${u}$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
subplot(3,1,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{u}$ [m/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
subplot(3,1,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{u} \; \rm{[m/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
saveas(H1,'./figures/CIN_Rodets_m.eps')

% Representar posició, velocitat i acceleració
H1=figure;font_size=18;
subplot(3,1,1);
plot(t_,x_/rE,'LineWidth',2)
ylabel('${\theta^E}$ [rad]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
subplot(3,1,2);
plot(t_,v_/rE,'LineWidth',2)
ylabel('$\dot{\theta^E}$ [rad/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
subplot(3,1,3);
plot(t_,a_/rE,'LineWidth',2)
ylabel('$\ddot{\theta^E} \; \rm{[rad/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 ttE])
saveas(H1,'./figures/CIN_RodetE_m.eps')



%% Resolució APARTAT b), definir trajectòria punt a punt del gir
t_ = [0:0.01:tt];

ddphi_max  = a_max;

phi(1) = 0;   % graus, inici
phi(2) = x/2-xE/2;  % graus, a x22
phi(3) = x/2+xE/2;  % graus, a x33
phi(4) = x;  % graus, final

t22 = tt/2 - tE/2;
t33 = tt/2 + tE/2;

td12 = t22-0;   % segons, durada primer bloc
td23 = t33-t22; % segons, durada segon bloc
td34 = tt -t33; % segons, durada tercer bloc

% Paràmetres trams
% tram de 1 a 2 (inicial)
if (phi(2)-phi(1))==0
    ddphi(1) = 0;
    t(1) = 0;
else
    ddphi(1) =  (phi(2)-phi(1))/abs(phi(2)-phi(1))*ddphi_max;
    t(1) = td12 - (td12^2-2*(phi(2)-phi(1))/ddphi(1))^0.5;
end
dphi12 = (phi(2)-phi(1))/(td12-1/2*t(1));

% tram de 3 a 4 (final)
if (phi(3)-phi(4))==0
    ddphi(4) = 0;
    t(4) = 0;
else
    ddphi(4) = (phi(3)-phi(4))/abs(phi(4)-phi(3))*ddphi_max;
    t(4) = td34 - (td34^2+2*(phi(4)-phi(3))/ddphi(4))^0.5;
end
dphi34 = (phi(4)-phi(3))/(td34-1/2*t(4));

% VELOCITATS: tram velocitat constant de 2 a 3
dphi23 = (phi(3)-phi(2))/td23;

% ACCELERACIONS i INTERVALS TEMPS: tram 2 i tram 3
if (dphi23-dphi12)==0
    ddphi(2) = 0;
    t(2) = 0;
else
    ddphi(2) = (dphi23-dphi12)/abs(dphi12-dphi23)*ddphi_max;
    t(2) = (dphi23-dphi12)/ddphi(2);
end
if (dphi34-dphi23)==0
    ddphi(3) = 0;
    t(3) = 0;
else
    ddphi(3) = (dphi34-dphi23)/abs(dphi23-dphi34)*ddphi_max;
    t(3) = (dphi34-dphi23)/ddphi(3);
end

% intervals de temps dels blocs
t12 = td12 - t(1) - 1/2*t(2);
t23 = td23 - 1/2*(t(2)+t(3));
t34 = td34 - t(4) - 1/2*t(3);
ttotal_test3 =sum(t) + t12+t23+t34;

% intervals de POSICIÓ
phii(1) = 0     *t(1) + 1/2*ddphi(1)*t(1)^2;
phii(2) = dphi12*t(2) + 1/2*ddphi(2)*t(2)^2;
phii(3) = dphi23*t(3) + 1/2*ddphi(3)*t(3)^2;
phii(4) = dphi34*t(4) + 1/2*ddphi(4)*t(4)^2;

phi12 = dphi12*t12;
phi23 = dphi23*t23;
phi34 = dphi34*t34;
phi_total3 = sum(phii) + phi12+phi23+phi34;

% 		Tram &  Tipus  & Duració  &  Despl.  &Velocitat & Acceleració    \\
% 		$(i)$ &   & $t$[ s] & ${x}$ [graus]& $\dot{\varphi}$ [graus/s] & $\ddot{x}$ [graus/s\tss{2}]    \\
disp(['Tram 1 , MUA',', t_1 =',num2str(t(1),'%10.3f'),'s , phi_1 =',num2str(phii(1),'%10.3f') 'graus,          a_1 =',num2str(ddphi(1),'%10.1f'),'graus/s2' ])
disp(['Tram 12, MU ',', t_12=',num2str(t12,'%10.3f'),'s , phi_12=',num2str(phi12,'%10.3f') 'graus, v_12=',num2str(dphi12,'%10.1f'),'graus/s' ])
disp(['Tram 2 , MUA',', t_2 =',num2str(t(2),'%10.3f'),'s , phi_2 =',num2str(phii(2),'%10.3f') 'graus,          a_2 =',num2str(ddphi(2),'%10.1f'),'graus/s2' ])
disp(['Tram 23, MU ',', t_23=',num2str(t23,'%10.3f'),'s , phi_23=',num2str(phi23,'%10.3f') 'graus, v_23=',num2str(dphi23,'%10.1f'),'graus/s' ])
disp(['Tram 3 , MUA',', t_3 =',num2str(t(3),'%10.3f'),'s , phi_3 =',num2str(phii(3),'%10.3f') 'graus,          a_3 =',num2str(ddphi(3),'%10.1f'),'graus/s2' ])
disp(['Tram 34, MU ',', t_34=',num2str(t34,'%10.3f'),'s , phi_34=',num2str(phi34,'%10.3f') 'graus, v_34=',num2str(dphi34,'%10.1f'),'graus/s' ])
disp(['Tram 4 , MUA',', t_4 =',num2str(t(4),'%10.3f'),'s , phi_4 =',num2str(phii(4),'%10.3f') 'graus,          a_4 =',num2str(ddphi(4),'%10.1f'),'graus/s2' ])
disp(' ')
disp(['temps total =',num2str(ttotal_test3,'%10.3f'),'s , phi total =',num2str(abs(phi_total3),'%10.3f'), 'graus'])

omg_R = dphi12/rR
omg_m = omg_R*i


% Definir trams de moviment: motion control
t_a(1)=      t(1);
t_a(2)=t_a(1)+t12;
t_a(3)=t_a(2)+t(2);
t_a(4)=t_a(3)+t23;
t_a(5)=t_a(4)+t(3);
t_a(6)=t_a(5)+t34;
t_a(7)=t_a(6)+t(4);

phi_a(1)=         phii(1);
phi_a(2)=phi_a(1)+phi12;
phi_a(3)=phi_a(2)+phii(2);
phi_a(4)=phi_a(3)+phi23;
phi_a(5)=phi_a(4)+phii(3);
phi_a(6)=phi_a(5)+phi34;
phi_a(7)=phi_a(6)+phii(4);

% Obtenir la trajectoria posicio, velocitat i acceleració
phi_   = zeros(1,length(t_));
dphi_  = zeros(1,length(t_));
ddphi_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= t_a(1)
        phi_(ii) = 0 + 1/2*ddphi(1)*t_(ii)^2;
        dphi_(ii) = ddphi(1)*t_(ii);
        ddphi_(ii) = ddphi(1);
    elseif  t_(ii) <= t_a(2)
        phi_(ii) = phi_a(1) + dphi12*(t_(ii)-t_a(1));
        dphi_(ii) = dphi12;
        ddphi_(ii) = 0;
    elseif    t_(ii) <= t_a(3)
        phi_(ii) = phi_a(2) + dphi12*(t_(ii)-t_a(2)) + 1/2*ddphi(2)*(t_(ii)-t_a(2))^2;
        dphi_(ii) = dphi12 + ddphi(2)*(t_(ii)-t_a(2));
        ddphi_(ii) = ddphi(2);
    elseif  t_(ii) <= t_a(4)
        phi_(ii) = phi_a(3) + dphi23*(t_(ii)-t_a(3));
        dphi_(ii) = dphi23;
        ddphi_(ii) = 0;
    elseif    t_(ii) <= t_a(5)
        phi_(ii) = phi_a(4) + dphi23*(t_(ii)-t_a(4)) + 1/2*ddphi(3)*(t_(ii)-t_a(4))^2;
        dphi_(ii) = dphi23 + ddphi(3)*(t_(ii)-t_a(4));
        ddphi_(ii) = ddphi(3);
    elseif  t_(ii) <= t_a(6)
        phi_(ii) = phi_a(5) + dphi34*(t_(ii)-t_a(5));
        dphi_(ii) = dphi34;
        ddphi_(ii) = 0;
    else
        phi_(ii) = phi_a(6) + dphi34*(t_(ii)-t_a(6)) + 1/2*ddphi(4)*(t_(ii)-t_a(6))^2;
        dphi_(ii) = dphi34 + ddphi(4)*(t_(ii)-t_a(6));
        ddphi_(ii) = ddphi(4);
    end
end

% Representar gir, velocitat i acceleració
H2=figure;
subplot(3,1,1);
plot(t_,phi_,'LineWidth',2)
ylabel('$u$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,2);
plot(t_,dphi_,'LineWidth',2)
ylabel('$\dot{u}$ [m/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,3);
plot(t_,ddphi_,'LineWidth',2)
ylabel('$\ddot{u} \; \rm{[m/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
saveas(H2,'./figures/CIN_Rodets_b.eps')

% Representar gir, velocitat i acceleració
H3=figure;
subplot(3,1,1);
plot(t_,phi_*i_t,'LineWidth',2)
ylabel('$\theta_m$ [rad]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,2);
plot(t_,dphi_*i_t,'LineWidth',2)
ylabel('$\dot{\theta}_m$ [rad/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,3);
plot(t_,ddphi_*i_t,'LineWidth',2)
ylabel('$\ddot{\theta}_m \; \rm{[rad/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
saveas(H3,'./figures/CIN_Motor_b.eps')
