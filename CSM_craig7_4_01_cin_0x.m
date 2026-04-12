%% Sol·lució implementada Exemple 1.4 (exemple del Craig)
% Generació de trajectòries
% J.A.Mayugo, UdG, 2021

close all; clear;

%% DADES

tt =  6; % s, temps finals
t22 = 2; % s, temps a 35 graus
t33 = 3; % s, temps a 25 graus

% Definint 4 punts

theta(1) = 10;  % graus, inici
theta(2) = 35;  % graus, a t = 2 s
theta(3) = 25;  % graus, a t = 2 + 1 = 3 s
theta(4) = 10;  % graus, final

ddtheta_max = 50; % graus/s2, acceleració màxima 

%% Resolució APARTAT, definir trajectòria punt a punt del gir

td12 = t22-0;   % segons, durada primer bloc
td23 = t33-t22; % segons, durada segon bloc
td34 = tt -t33; % segons, durada tercer bloc

t_ = [0:1/100:tt];%        % segons, discretització domini temporal a 100Hz

%% Obtenir trajectòria sense enllaços (trajectòria ideal)
theta_ideal_=zeros(1,length(t_));
for ii = [1:length(t_)] % phii_, graus, orientació ideal a cada instant
    if t_(ii) <= t22
        theta_ideal_(ii) = theta(1) + (theta(2)-theta(1))/td12*(t_(ii)-0);
    elseif t_(ii) <= t33
        theta_ideal_(ii) = theta(2) + (theta(3)-theta(2))/td23*(t_(ii)-t22);
    else
        theta_ideal_(ii) = theta(3) + (theta(4)-theta(3))/td34*(t_(ii)-t33);
    end
end

% Paràmetres trams
% tram de 1 a 2 (inicial)
if (theta(2)-theta(1))==0
    ddtheta(1) = 0;
    t(1) = 0;
else
    ddtheta(1) =  (theta(2)-theta(1))/abs(theta(2)-theta(1))*ddtheta_max;
    t(1) = td12 - (td12^2-2*(theta(2)-theta(1))/ddtheta(1))^0.5;
end
dtheta12 = (theta(2)-theta(1))/(td12-1/2*t(1));

% tram de 3 a 4 (final)
if (theta(3)-theta(4))==0
    ddtheta(4) = 0;
    t(4) = 0;
else
    ddtheta(4) = (theta(3)-theta(4))/abs(theta(4)-theta(3))*ddtheta_max;
    t(4) = td34 - (td34^2+2*(theta(4)-theta(3))/ddtheta(4))^0.5;
end
dtheta34 = (theta(4)-theta(3))/(td34-1/2*t(4));

% VELOCITATS: tram velocitat constant de 2 a 3
dtheta23 = (theta(3)-theta(2))/td23;

% ACCELERACIONS i INTERVALS TEMPS: tram 2 i tram 3
if (dtheta23-dtheta12)==0
    ddtheta(2) = 0;
    t(2) = 0;
else
    ddtheta(2) = (dtheta23-dtheta12)/abs(dtheta12-dtheta23)*ddtheta_max;
    t(2) = (dtheta23-dtheta12)/ddtheta(2);
end
if (dtheta34-dtheta23)==0
    ddtheta(3) = 0;
    t(3) = 0;
else
    ddtheta(3) = (dtheta34-dtheta23)/abs(dtheta23-dtheta34)*ddtheta_max;
    t(3) = (dtheta34-dtheta23)/ddtheta(3);
end

% intervals de temps dels blocs
t12 = td12 - t(1) - 1/2*t(2);
t23 = td23 - 1/2*(t(2)+t(3));
t34 = td34 - t(4) - 1/2*t(3);
ttotal_test3 =sum(t) + t12+t23+t34;

% intervals de POSICIÓ
thetai(1) = 0     *t(1) + 1/2*ddtheta(1)*t(1)^2;
thetai(2) = dtheta12*t(2) + 1/2*ddtheta(2)*t(2)^2;
thetai(3) = dtheta23*t(3) + 1/2*ddtheta(3)*t(3)^2;
thetai(4) = dtheta34*t(4) + 1/2*ddtheta(4)*t(4)^2;

theta12 = dtheta12*t12;
theta23 = dtheta23*t23;
theta34 = dtheta34*t34;
theta_total3 = sum(thetai) + theta12+theta23+theta34;

% 		Tram &  Tipus  & Duració  &  Despl.  &Velocitat & Acceleració    \\
% 		$(i)$ &   & $t$[ s] & ${x}$ [graus]& $\dot{\varphi}$ [graus/s] & $\ddot{x}$ [graus/s\tss{2}]    \\
disp(['Tram 1 , MUA',', t_1 =',num2str(t(1),'%10.3f'),'s , phi_1 =',num2str(thetai(1),'%10.3f') 'graus,          a_1 =',num2str(ddtheta(1),'%10.1f'),'graus/s2' ])
disp(['Tram 12, MU ',', t_12=',num2str(t12,'%10.3f'),'s , phi_12=',num2str(theta12,'%10.3f') 'graus, v_12=',num2str(dtheta12,'%10.1f'),'graus/s' ])
disp(['Tram 2 , MUA',', t_2 =',num2str(t(2),'%10.3f'),'s , phi_2 =',num2str(thetai(2),'%10.3f') 'graus,          a_2 =',num2str(ddtheta(2),'%10.1f'),'graus/s2' ])
disp(['Tram 23, MU ',', t_23=',num2str(t23,'%10.3f'),'s , phi_23=',num2str(theta23,'%10.3f') 'graus, v_23=',num2str(dtheta23,'%10.1f'),'graus/s' ])
disp(['Tram 3 , MUA',', t_3 =',num2str(t(3),'%10.3f'),'s , phi_3 =',num2str(thetai(3),'%10.3f') 'graus,          a_3 =',num2str(ddtheta(3),'%10.1f'),'graus/s2' ])
disp(['Tram 34, MU ',', t_34=',num2str(t34,'%10.3f'),'s , phi_34=',num2str(theta34,'%10.3f') 'graus, v_34=',num2str(dtheta34,'%10.1f'),'graus/s' ])
disp(['Tram 4 , MUA',', t_4 =',num2str(t(4),'%10.3f'),'s , phi_4 =',num2str(thetai(4),'%10.3f') 'graus,          a_4 =',num2str(ddtheta(4),'%10.1f'),'graus/s2' ])
disp(' ')
disp(['temps total =',num2str(ttotal_test3,'%10.3f'),'s , phi total =',num2str(abs(theta_total3),'%10.3f'), 'graus'])

% Definir trams de moviment: motion control
t_a(1)=      t(1);
t_a(2)=t_a(1)+t12;
t_a(3)=t_a(2)+t(2);
t_a(4)=t_a(3)+t23;
t_a(5)=t_a(4)+t(3);
t_a(6)=t_a(5)+t34;
t_a(7)=t_a(6)+t(4);

theta_a(1)= theta(1) +        thetai(1);
theta_a(2)=theta_a(1)+theta12;
theta_a(3)=theta_a(2)+thetai(2);
theta_a(4)=theta_a(3)+theta23;
theta_a(5)=theta_a(4)+thetai(3);
theta_a(6)=theta_a(5)+theta34;
theta_a(7)=theta_a(6)+thetai(4);

% Obtenir la trajectoria posicio, velocitat i acceleració
theta_   = zeros(1,length(t_));
dtheta_  = zeros(1,length(t_));
ddtheta_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= t_a(1)
        theta_(ii) = theta(1) + 1/2*ddtheta(1)*t_(ii)^2;
        dtheta_(ii) = ddtheta(1)*t_(ii);
        ddtheta_(ii) = ddtheta(1);
    elseif  t_(ii) <= t_a(2)
        theta_(ii) = theta_a(1) + dtheta12*(t_(ii)-t_a(1));
        dtheta_(ii) = dtheta12;
        ddtheta_(ii) = 0;
    elseif    t_(ii) <= t_a(3)
        theta_(ii) = theta_a(2) + dtheta12*(t_(ii)-t_a(2)) + 1/2*ddtheta(2)*(t_(ii)-t_a(2))^2;
        dtheta_(ii) = dtheta12 + ddtheta(2)*(t_(ii)-t_a(2));
        ddtheta_(ii) = ddtheta(2);
    elseif  t_(ii) <= t_a(4)
        theta_(ii) = theta_a(3) + dtheta23*(t_(ii)-t_a(3));
        dtheta_(ii) = dtheta23;
        ddtheta_(ii) = 0;
    elseif    t_(ii) <= t_a(5)
        theta_(ii) = theta_a(4) + dtheta23*(t_(ii)-t_a(4)) + 1/2*ddtheta(3)*(t_(ii)-t_a(4))^2;
        dtheta_(ii) = dtheta23 + ddtheta(3)*(t_(ii)-t_a(4));
        ddtheta_(ii) = ddtheta(3);
    elseif  t_(ii) <= t_a(6)
        theta_(ii) = theta_a(5) + dtheta34*(t_(ii)-t_a(5));
        dtheta_(ii) = dtheta34;
        ddtheta_(ii) = 0;
    else
        theta_(ii) = theta_a(6) + dtheta34*(t_(ii)-t_a(6)) + 1/2*ddtheta(4)*(t_(ii)-t_a(6))^2;
        dtheta_(ii) = dtheta34 + ddtheta(4)*(t_(ii)-t_a(6));
        ddtheta_(ii) = ddtheta(4);
    end
end

% Representar gir, velocitat i acceleració
H1=figure;font_size=18;hold on;
plot(t_,theta_,'LineWidth',2)
plot(t_,theta_ideal_,'--','LineWidth',1)
plot([0 t22 t33 tt],theta,'ro','LineWidth',2,'MarkerSize',8)
plot([0 t22 t33 tt],theta,'r.','LineWidth',2,'MarkerSize',8)
ylabel('$\theta$ [graus]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
H1.Position = [100 100 560 240];
saveas(H1,'./figures/CSM_craig7_4_01_cin_a.svg')
saveas(H1,'./figures/CSM_craig7_4_01_cin_a.pdf')

% Representar gir, velocitat i acceleració
H2=figure;font_size=18;
subplot(2,1,1);
plot(t_,dtheta_,'LineWidth',2)
ylabel('$\dot{\theta}$ [graus/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
subplot(2,1,2);
plot(t_,ddtheta_,'LineWidth',2)
ylabel('$\ddot{\theta} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 tt])
H2.Position = [100 100 560 140*2+20];
saveas(H2,'./figures/CSM_craig7_4_01_cin_b.svg')
saveas(H2,'./figures/CSM_craig7_4_01_cin_b.pdf')
