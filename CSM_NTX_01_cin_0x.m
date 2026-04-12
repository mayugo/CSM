%% Sol·lució implementada NTX trajectòries
% Generació de trajectòries
% J.A.Mayugo, UdG, 2021

close all, clear
%% DADES

% mides
W = 0.112;  %,m amplada vehicle
R = 0.028;  %,m radi roda
D = 0.045;  %,m  posicio cdm

% relació de transmissió reductor
i_r = 10;

% dades APARTAT a)
a_max   =  0.1;     % m/s2, acceleració màxima vehicle
tt      = 20.0;     % segons, temps total moviment
x       =  2.5;     % m, recorregut total vehicle

% dades APARTAT c), definint 5 punts
ddphi_max = 60.0; % graus/s2, acceleració màxima en l'orientació 

x22 = 0.8;  % m, posició punt angle referència phi2
x33 = 1.2;  % m, posició punt angle referència phi3
x44 = 2;    % m, posició punt angle referència phi4

phi1 =   0;     % graus, inici
phi2 =  90;     % graus, a x22
phi3 = -45;     % graus, a x33
phi4 =   0;     % graus, a x44
phi5 =   0;     % graus, final 

%% Obtenir trajectòria sense enllaços (trajectòria ideal)
t_ = [0:1/20:tt];       % segons, discretització domini temporal a 20Hz
x_ideal_ = x/tt*t_;     % m, trajectòria ideal a velocitat constant
for ii = [1:length(t_)] % phii_, graus, orientació ideal a cada instant
    if     x_ideal_(ii) <= x22
        phi_ideal_(ii) = phi1 + (phi2-phi1)*(x_ideal_(ii)-0)/x22;
    elseif x_ideal_(ii) <= x33
        phi_ideal_(ii) = phi2 + (phi3-phi2)*(x_ideal_(ii)-x22)/(x33-x22);
    elseif x_ideal_(ii) <= x44
        phi_ideal_(ii) = phi3 + (phi4-phi3)*(x_ideal_(ii)-x33)/(x44-x33);
    else
        phi_ideal_(ii) = phi4 + (phi5-phi4)*(x_ideal_(ii)-x44)/(x-x44);
    end
end
[c0 i_x(1)] = min(abs(x_ideal_-0));
[c0 i_x(2)] = min(abs(x_ideal_-x22));
[c0 i_x(3)] = min(abs(x_ideal_-x33));
[c0 i_x(4)] = min(abs(x_ideal_-x44));
[c0 i_x(5)] = min(abs(x_ideal_-x));
x_input = x_ideal_([i_x]);


%% Resolució APATAT a) marxa-moviment-parada
t_b = tt/2 - (a_max^2*tt^2-4*a_max*x)^0.5/2/a_max; % temps acc. i frenada
x_b = 1/2*a_max*t_b^2;  % desplaçament en zona de t_b
v_max = t_b*a_max;      % velocitat màxima

% Definir trams de moviment: motion control
tx_a(1) = t_b;
tx_a(2) = tx_a(1) + tt-2*t_b;
tx_a(3) = tx_a(2) + t_b;

x_a(1) =       + 1/2*a_max*tx_a(1)^2;
x_a(2) = x_a(1) + v_max*(tx_a(2)-tx_a(1));
x_a(3) = x_a(2) + v_max*(tx_a(3)-tx_a(2)) - 1/2*a_max*(tx_a(3)-tx_a(2))^2;

% Obtenir la trajectoria posicio, velocitat i acceleració
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= tx_a(1)
        x_(ii) = + 1/2*a_max*t_(ii)^2;
        v_(ii) = a_max*t_(ii);
        a_(ii) = a_max;
    elseif  t_(ii) <= tx_a(2)
        x_(ii) = x_a(1) + v_max*(t_(ii)-tx_a(1));
        v_(ii) = v_max;
        a_(ii) = 0;
    else %if  t_(ii) <= ti(3)
        x_(ii) = x_a(2) + v_max*(t_(ii)-tx_a(2)) - 1/2*a_max*(t_(ii)-tx_a(2))^2;
        v_(ii) = v_max - a_max*(t_(ii)-tx_a(2));
        a_(ii) = -a_max;
    end
end

%  Tram & tipus & $t$ [s] & $u$ [m] & $\dot{u}$ [m/s] & $\ddot{u}$ [m/s\tss{2}]]    \\
disp(['Tram 1 , 0 a 1, acceleració',', t_1 =',num2str(t_b,'%10.3f'),'s , u_1 =',num2str(x_a(1),'%10.3f'),'m , a_max =',num2str(a_max,'%10.3f') 'm/s2' ])
disp(['Tram 2 , 1 a 2, vel. constant',', t_2 =',num2str(tt-2*t_b,'%10.3f'),'s , u_2 =',num2str(x_a(2)-x_a(1),'%10.3f'),'m , v_max =',num2str(v_max,'%10.3f') 'm/s' ])
disp(['Tram 3 , 2 a 3, desacceleració',', t_3 =',num2str(t_b,'%10.3f'),'s , u_3 =',num2str(x_a(3)-x_a(2),'%10.3f'),'m , a_max =',num2str(-a_max,'%10.3f') 'm/s2' ])
disp(' ')
disp(['temps total =',num2str(tt,'%10.2f'),'s , x total =',num2str(abs(x),'%10.2f'), 'mm'])

% Representar poció, velocitat i acceleració
H1=figure;font_size=18;
subplot(3,1,1);
plot(t_,x_,'LineWidth',2)
ylabel('${u}$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{u}$ [m/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{u} \; \rm{[m/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
saveas(H1,'./figures/CIN_NTX_x.eps')

%% Resolució APARTAT c), definir trajectòria punt a punt del gir
% Temps associats als canvis en l'orientació
t22 = t_b + (x22-x_b)/(v_max); % temps necessari per arribar a x22
t33=  t_b + (x33-x_b)/(v_max); % temps necessari per arribar a x33
t44=  t_b + (x44-x_b)/(v_max); % temps necessari per arribar a x44
    
td12 = t22-0;   % segons, durada primer bloc
td23=  t33-t22; % segons, durada segon bloc
td34=  t44-t33; % segons, durada tercer bloc
td45 = tt -t44; % segons, durada quart bloc

% Paràmetres trams
% tram de 1 a 2 (inicial)
ddphi1 =  (phi2-phi1)/abs(phi2-phi1)*ddphi_max;
t1 = td12 - (td12^2-2*(phi2-phi1)/ddphi1)^0.5;
dphi12 = (phi2-phi1)/(td12-1/2*t1);

% tram de 4 a 5 (final)
if (phi4-phi5)==0
    ddphi5 = 0;
    t5 = 0;
else
    ddphi5 = (phi4-phi5)/abs(phi5-phi4)*ddphi_max;
    t5 = td45 - (td45^2+2*(phi5-phi4)/ddphi5)^0.5;
end
dphi45 = (phi5-phi4)/(td45-1/2*t5);

% VELOCITATS: tram de 2 a 3 i de 3 a 4
dphi23 = (phi3-phi2)/td23;
dphi34 = (phi4-phi3)/td34;

% ACCELERACIONS: trams 2, 3 i 4
ddphi2 = (dphi23-dphi12)/abs(dphi12-dphi23)*ddphi_max;
ddphi3 = (dphi34-dphi23)/abs(dphi23-dphi34)*ddphi_max;
ddphi4 = (dphi45-dphi34)/abs(dphi34-dphi45)*ddphi_max;

% intervals de temps
t2 = (dphi23-dphi12)/ddphi2;
t3 = (dphi34-dphi23)/ddphi3;
t4 = (dphi45-dphi34)/ddphi4;
t12 = td12 - t1 - 1/2*t2;
t23 = td23 - 1/2*(t2+t3);
t34 = td34 - 1/2*(t3+t4);
t45 = td45  - t5 - 1/2*t4;
ttotal2=t1+t2+t3+t4+t5+t12+t23+t34+t45

% intervals de POSICIÓ
phii1 = 0 *t1  + 1/2*ddphi1*t1^2;
phii2 = dphi12*t2 + 1/2*ddphi2*t2^2;
phii3 = dphi23*t3 + 1/2*ddphi3*t3^2;
phii4 = dphi34*t4 + 1/2*ddphi4*t4^2;
phii5 = dphi45*t5 + 1/2*ddphi5*t5^2;

phi12 = dphi12*t12;
phi23 = dphi23*t23;
phi34 = dphi34*t34;
phi45 = dphi45*t45;
phi_total2 = phii1+phii2+phii3+phii4+phii5 + phi12+phi23+phi34+phi45

% 		Tram &  Tipus  & Duració  &  Despl.  &Velocitat & Acceleració    \\
% 		$(i)$ &   & $t$[ s] & ${x}$ [graus]& $\dot{\varphi}$ [graus/s] & $\ddot{x}$ [graus/s\tss{2}]    \\
disp(['Tram 1 , MUA',', t_1 =',num2str(t1,'%10.3f'),'s , phi_1 =',num2str(phii1,'%10.3f') 'graus,          a_1 =',num2str(ddphi1,'%10.1f'),'graus/s2' ])
disp(['Tram 12, MU ',', t_12=',num2str(t12,'%10.3f'),'s , x_12=',num2str(phi12,'%10.3f') 'graus, v_12=',num2str(dphi12,'%10.1f'),'graus/s' ])
disp(['Tram 2 , MUA',', t_2 =',num2str(t2,'%10.3f'),'s , x_2 =',num2str(phii2,'%10.3f') 'graus,          a_2 =',num2str(ddphi2,'%10.1f'),'graus/s2' ])
disp(['Tram 23, MU ',', t_23=',num2str(t23,'%10.3f'),'s , x_23=',num2str(phi23,'%10.3f') 'graus, v_23=',num2str(dphi23,'%10.1f'),'graus/s' ])
disp(['Tram 3 , MUA',', t_3 =',num2str(t3,'%10.3f'),'s , x_3 =',num2str(phii3,'%10.3f') 'graus,          a_3 =',num2str(ddphi3,'%10.1f'),'graus/s2' ])
disp(['Tram 34, MU ',', t_34=',num2str(t34,'%10.3f'),'s , x_34=',num2str(phi34,'%10.3f') 'graus, v_34=',num2str(dphi34,'%10.1f'),'graus/s' ])
disp(['Tram 4 , MUA',', t_4 =',num2str(t4,'%10.3f'),'s , x_4 =',num2str(phii4,'%10.3f') 'graus,          a_4 =',num2str(ddphi4,'%10.1f'),'graus/s2' ])
disp(['Tram 45, MU ',', t_45=',num2str(t45,'%10.3f'),'s , x_45=',num2str(phi45,'%10.3f') 'graus, v_45=',num2str(dphi45,'%10.1f'),'graus/s' ])
disp(['Tram 5 , MUA',', t_5 =',num2str(t5,'%10.3f'),'s , x_5 =',num2str(phii5,'%10.3f') 'graus,          a_5 =',num2str(ddphi5,'%10.1f'),'graus/s2' ])
disp(' ')
disp(['temps total = =',num2str(ttotal2,'%10.3f'),'s , x total =',num2str(abs(phi_total2),'%10.3f'), 'graus'])

% Definir trams de moviment: motion control
t_a(1)=      t1;
t_a(2)=t_a(1)+t12;
t_a(3)=t_a(2)+t2;
t_a(4)=t_a(3)+t23;
t_a(5)=t_a(4)+t3;
t_a(6)=t_a(5)+t34;
t_a(7)=t_a(6)+t4;
t_a(8)=t_a(7)+t45;
t_a(9)=t_a(8)+t5;

phi_a(1)=      phii1;
phi_a(2)=phi_a(1)+phi12;
phi_a(3)=phi_a(2)+phii2;
phi_a(4)=phi_a(3)+phi23;
phi_a(5)=phi_a(4)+phii3;
phi_a(6)=phi_a(5)+phi34;
phi_a(7)=phi_a(6)+phii4;
phi_a(8)=phi_a(7)+phi45;
phi_a(9)=phi_a(8)+phii5;

% Obtenir la trajectoria posicio, velocitat i acceleració
phi_ = zeros(1,length(t_));
dphi_ = zeros(1,length(t_));
ddphi_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= t_a(1)
        phi_(ii) = 0 + 1/2*ddphi1*t_(ii)^2;
        dphi_(ii) = ddphi1*t_(ii);
        ddphi_(ii) = ddphi1;
    elseif  t_(ii) <= t_a(2)
        phi_(ii) = phi_a(1) + dphi12*(t_(ii)-t_a(1));
        dphi_(ii) = dphi12;
        ddphi_(ii) = 0;
    elseif    t_(ii) <= t_a(3)
        phi_(ii) = phi_a(2) + dphi12*(t_(ii)-t_a(2)) + 1/2*ddphi2*(t_(ii)-t_a(2))^2;
        dphi_(ii) = dphi12 + ddphi2*(t_(ii)-t_a(2));
        ddphi_(ii) = ddphi2;
    elseif  t_(ii) <= t_a(4)
        phi_(ii) = phi_a(3) + dphi23*(t_(ii)-t_a(3));
        dphi_(ii) = dphi23;
        ddphi_(ii) = 0;
    elseif    t_(ii) <= t_a(5)
        phi_(ii) = phi_a(4) + dphi23*(t_(ii)-t_a(4)) + 1/2*ddphi3*(t_(ii)-t_a(4))^2;
        dphi_(ii) = dphi23 + ddphi3*(t_(ii)-t_a(4));
        ddphi_(ii) = ddphi3;
    elseif  t_(ii) <= t_a(6)
        phi_(ii) = phi_a(5) + dphi34*(t_(ii)-t_a(5));
        dphi_(ii) = dphi34;
        ddphi_(ii) = 0;
    elseif  t_(ii) <= t_a(7)
        phi_(ii) = phi_a(6) + dphi34*(t_(ii)-t_a(6)) + 1/2*ddphi4*(t_(ii)-t_a(6))^2;
        dphi_(ii) = dphi34 + ddphi4*(t_(ii)-t_a(6));
        ddphi_(ii) = ddphi4;        
    elseif  t_(ii) <= t_a(8)
        phi_(ii) = phi_a(7) + dphi45*(t_(ii)-t_a(7));
        dphi_(ii) = dphi45;
        ddphi_(ii) = 0;
    else
        phi_(ii) = phi_a(8) + dphi45*(t_(ii)-t_a(8)) + 1/2*ddphi5*(t_(ii)-t_a(8))^2;
        dphi_(ii) = dphi45 + ddphi4*(t_(ii)-t_a(8));
        ddphi_(ii) = ddphi5;
    end
end

% Representar gir, velocitat i acceleració
H2=figure;
subplot(3,1,1);
plot(t_,phi_,'LineWidth',2)
ylabel('$\varphi$ [graus]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,2);
plot(t_,dphi_,'LineWidth',2)
ylabel('$\dot{\varphi}$ [graus/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,3);
plot(t_,ddphi_,'LineWidth',2)
ylabel('$\ddot{\varphi} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
saveas(H2,'./figures/CIN_NTX_phi.eps')

% Representar gir, velocitat i acceleració en funció de x
H20=figure;
subplot(3,1,1);
plot(x_,phi_,'LineWidth',2)
ylabel('$\varphi$ [graus]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 x])
subplot(3,1,2);
plot(x_,dphi_,'LineWidth',2)
ylabel('$\dot{\varphi}$ [graus/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 x])
subplot(3,1,3);
plot(x_,ddphi_,'LineWidth',2)
ylabel('$\ddot{\varphi} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$x$ [m]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 x])

% Cinemàtica inversa: girs en els motor
theta_r_  = 1/R * x_ + W/(2*R)* phi_*pi/180;
theta_l_  = 1/R * x_ - W/(2*R)* phi_*pi/180;
dtheta_r_ = 1/R * v_ + W/(2*R)*dphi_*pi/180;
dtheta_l_ = 1/R * v_ - W/(2*R)*dphi_*pi/180;
ddtheta_r_ = 1/R * a_ + W/(2*R)*ddphi_*pi/180;
ddtheta_l_ = 1/R * a_ - W/(2*R)*ddphi_*pi/180;

theta_m_r_= i_r *theta_r_;
theta_m_l_= i_r *theta_l_;
dtheta_m_r_= i_r *dtheta_r_;
dtheta_m_l_= i_r *dtheta_l_;
ddtheta_m_r_= i_r *ddtheta_r_;
ddtheta_m_l_= i_r *ddtheta_l_;

% Representar omega motor
H21=figure;font_size=18;
subplot(3,1,1);hold on;
plot(t_,theta_m_r_,'LineWidth',2)
plot(t_,theta_m_l_,'--','LineWidth',2)
ylabel('$\theta_m$ [rad]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
legend('Motor r','Motor l','Location','Best')
subplot(3,1,2);hold on;
plot(t_,dtheta_m_r_,'LineWidth',2)
plot(t_,dtheta_m_l_,'--','LineWidth',2)
ylabel('$\dot{\theta}_m$ [rad/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(3,1,3);hold on;
plot(t_,ddtheta_m_r_,'LineWidth',2)
plot(t_,ddtheta_m_l_,'--','LineWidth',2)
ylabel('$\ddot{\theta}_m$ [rad/s$^2$]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
saveas(H21,'./figures/CIN_NTX_omg.eps')

inc_x_ = diff(x_);
inc_x_ideal_ = diff(x_ideal_);
inc_phi_ = diff(phi_*pi/180);               % increment angle amb radians
inc_phi_ideal_ = diff(phi_ideal_*pi/180);

for ii = [1:length(x_)-1]
    if inc_phi_(ii)==0      % estem en una zona sense curvatura
                C_(ii) = inc_x_(ii);
                C_ideal_(ii) = inc_x_ideal_(ii);
    else                    % zona amb curvatura
                C_(ii)       = 2*sin(inc_phi_(ii)/2)*inc_x_(ii)/inc_phi_(ii);
                C_ideal_(ii) = 2*sin(inc_phi_ideal_(ii)/2)*inc_x_ideal_(ii)/inc_phi_ideal_(ii);
    end    
end

for ii = [1:length(x_)]
    if ii == 1
        p_x(ii)=0;
        p_y(ii)=0;        
        p_ideal_x(ii)=0;
        p_ideal_y(ii)=0;
    else
        p_x(ii) = p_x(ii-1) + C_(ii-1)*cosd(phi_(ii));
        p_y(ii) = p_y(ii-1) + C_(ii-1)*sind(phi_(ii));  
        p_ideal_x(ii) = p_ideal_x(ii-1) + C_ideal_(ii-1)*cosd(phi_ideal_(ii));
        p_ideal_y(ii) = p_ideal_y(ii-1) + C_ideal_(ii-1)*sind(phi_ideal_(ii));  
    end
end
p_input_x = p_ideal_x([i_x]);
p_input_y = p_ideal_y([i_x]);


H33=figure;font_size=18;hold on
%plot(p_x,p_y,'LineWidth',2)
plot(p_ideal_x,p_ideal_y,'LineWidth',2,'Color','#D95319')
plot(p_input_x,p_input_y,'o','MarkerSize',10,...
    'MarkerEdgeColor','#D95319',...
    'MarkerFaceColor','#D95319')
gap = 0.04
text(p_input_x(2)-8.5*gap,p_input_y(2)+0.5*gap,{['$u=$',num2str(x22),'m,'],[ '$\varphi=$',num2str(phi2),'$^o$']},'Interpreter','latex','FontSize',font_size)
text(p_input_x(3)+gap,p_input_y(3)+gap,['$u=$',num2str(x33),'m, $\varphi=$',num2str(phi3),'$^o$'],'Interpreter','latex','FontSize',font_size)
text(p_input_x(4)-2*gap,p_input_y(4)-gap*3.5,{['$u=$',num2str(x44),'m,'],['$\varphi=$',num2str(phi4),'$^o$']},'Interpreter','latex','FontSize',font_size)
text(p_input_x(5)-2*gap,p_input_y(5)-gap*3.5,{['$u=$',num2str(x),'m,'],['$\varphi=$',num2str(phi5),'$^o$']},'Interpreter','latex','FontSize',font_size)
ylabel('$y$ [m]','Interpreter','latex','FontSize',font_size)
xlabel('$x$ [m]','Interpreter','latex','FontSize',font_size)
%legend('Real','Teòric','Location','Best')
set(gca,'FontSize',font_size)
axis equal
xlim([0 max(p_ideal_x)])
ylim([0 0.8])

