%% Sol·lució implementada trajectòria capçal
% Generació de trajectòries
% 01_cin_06.tex
% J.A.Mayugo, UdG, 2020

close all; clear;
%% DADES
t_total = 8.5;  % segons
a_max = 200;    % mm/s2

rP = 80/2;      % mm
i = 66;

%% Resolució: Métode 1, marxa-moviment-parada
% S'imposa un temps de treball i una velocitat de treball a l'anada
t_treball = 4;      % segons
v_treball = 100;    % mm/s

% Paràmetres cicle d'anada
tb_a    = v_treball/a_max;
xb_a    = 1/2*a_max*tb_a^2;

ta      = 2*tb_a + t_treball;
xa      = 2*xb_a + t_treball*v_treball;

% Paràmetres cicle de tornada
tt      = t_total - ta;
tb_t    = tt/2 - sqrt(a_max^2*tt^2-4*a_max*(xa))/2/a_max;
xb_t    = 1/2*a_max*tb_t^2;

v_max   = tb_t*a_max;

% En el motor:
omega_max       = v_max*i/rP
omega_treball   = v_treball*i/rP

n_max           = omega_max*60/(2*pi)
n_treball       = omega_treball*60/(2*pi)

% Definir trams de moviment: motion control
ti(1) = tb_a;
ti(2) = ti(1) + t_treball;
ti(3) = ti(2) + tb_a;
ti(4) = ti(3) + tb_t;
ti(5) = ti(4) + tt-2*tb_t;
ti(6) = ti(5) + tb_t;

xi(1) =       + 1/2*a_max*ti(1)^2;
xi(2) = xi(1) + v_treball*(ti(2)-ti(1));
xi(3) = xi(2) + v_treball*(ti(3)-ti(2)) - 1/2*a_max*(ti(3)-ti(2))^2;
xi(4) = xi(3) + 0*(ti(4)-ti(3)) - 1/2*a_max*(ti(4)-ti(3))^2;
xi(5) = xi(4) - v_max*(ti(5)-ti(4));
xi(6) = xi(5) - v_max*(ti(6)-ti(5)) + 1/2*a_max*(ti(6)-ti(5))^2;

% Obtenir la trajectoria posicio, velocitat i acceleració
t_ = [0:0.05:t_total];
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= ti(1)
        x_(ii) = + 1/2*a_max*t_(ii)^2;
        v_(ii) = a_max*t_(ii);
        a_(ii) = a_max;
    elseif  t_(ii) <= ti(2)
        x_(ii) = xi(1) + v_treball*(t_(ii)-ti(1));
        v_(ii) = v_treball;
        a_(ii) = 0;
    elseif  t_(ii) <= ti(3)
        x_(ii) = xi(2) + v_treball*(t_(ii)-ti(2)) - 1/2*a_max*(t_(ii)-ti(2))^2;
        v_(ii) = v_treball - a_max*(t_(ii)-ti(2));
        a_(ii) = -a_max;
    elseif  t_(ii) <= ti(4)
        x_(ii) = xi(3) + 0*(t_(ii)-ti(3)) - 1/2*a_max*(t_(ii)-ti(3))^2;
        v_(ii) = 0 - a_max*(t_(ii)-ti(3));
        a_(ii) = -a_max;
    elseif  t_(ii) <= ti(5)
        x_(ii) = xi(4) - v_max*(t_(ii)-ti(4));
        v_(ii) = -v_max;
        a_(ii) = 0;
    else
        x_(ii) = xi(5) - v_max*(t_(ii)-ti(5)) + 1/2*a_max*(t_(ii)-ti(5))^2;
        v_(ii) = -v_max + a_max*(t_(ii)-ti(5));
        a_(ii) = a_max;
    end
end

% Representar poció, velocitat i acceleració
H1=figure;font_size=18;
subplot(3,1,1);
plot(t_,x_,'LineWidth',2)
ylabel('${u}$ [mm]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{u}$ [mm/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{u} \; \rm{[mm/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
%saveas(H1,'./figures/CIN_Ex11_met1.svg')
%saveas(H1,'./figures/CIN_Ex11_met1.pdf')

% Representar omega motor
H11=figure;font_size=18;
plot(t_,v_*i/rP,'LineWidth',2)
ylabel('$\omega_m$ [rad/s]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
H11.Position = [100 100 560 240];
%saveas(H11,'./figures/CIN_Ex11_met1_omg.svg')
%saveas(H11,'./figures/CIN_Ex11_met1_omg.pdf')

%% Resolució: Métode 2, definir trajectòria punt a punt
% S'imposa arranc inicial, llavors un moviment de treball i un de tornada
% definint 4 punts

td12 = 0.4;  % segons
td23 = 4.9;  % segons
td34 = t_total-(td12+td23); 

x1 = 0;      % mm
x2 = 10;     % mm
x3 = 500;    % mm
x4 = 0;      % mm

% Paràmetres trams
% tram de 1 a 2 (inicial)
a1 =  (x2-x1)/abs(x2-x1)*a_max;
t1 = td12 - (td12^2-2*(x2-x1)/a1)^0.5;
v12 = (x2-x1)/(td12-1/2*t1);

% tram de 3 a 4 (final)
a4 = (x3-x4)/abs(x4-x3)*a_max;
t4 = td34 - (td34^2+2*(x4-x3)/a4)^0.5;
v34 = (x4-x3)/(td34-1/2*t4);

% VELOCITATS
% tram de 2 a 3
v23 = (x3-x2)/td23;

% ACCELERACIONS
% tram 2
a2 = (v23-v12)/abs(v12-v23)*a_max;
% tram 3
a3 = (v34-v23)/abs(v23-v34)*a_max;

% intervals de temps
t2 = (v23-v12)/a2;
t3 = (v34-v23)/a3;
t12 = td12 - t1 - 1/2*t2;
t23 = td23 - 1/2*(t2+t3);
t34 = td34  - t4 - 1/2*t3;
ttotal2=t1+t2+t3+t4+t12+t23+t34

% intervals de POSICIÓ
xi1 = 0 *t1  + 1/2*a1*t1^2;
xi2 = v12*t2 + 1/2*a2*t2^2;
xi3 = v23*t3 + 1/2*a3*t3^2;
xi4 = v34*t4 + 1/2*a4*t4^2;
x12 = v12*t12;
x23 = v23*t23;
x34 = v34*t34;
xtotal2 = xi1 + xi2+xi3+xi4+x12+x23+x34

% 		Tram &  Tipus  & Duració  &  Despl.  &Velocitat & Acceleració    \\
% 		$(i)$ &   & $t$[ s] & ${x}$ [mm]& $\dot{x}$ [mm/s] & $\ddot{x}$ [mm/s\tss{2}]    \\
disp(['Tram 1 , MUA',', t_1 =',num2str(t1,'%10.3f'),'s , x_1 =',num2str(xi1,'%10.3f') 'mm,          a_1 =',num2str(a1,'%10.f'),'mm/s2' ])
disp(['Tram 12, MU ',', t_12=',num2str(t12,'%10.3f'),'s , x_12=',num2str(x12,'%10.3f') 'mm, v_12=',num2str(v12,'%10.f'),'mm/s' ])
disp(['Tram 2 , MUA',', t_2 =',num2str(t2,'%10.3f'),'s , x_2 =',num2str(xi2,'%10.3f') 'mm,          a_2 =',num2str(a2,'%10.f'),'mm/s2' ])
disp(['Tram 23, MU ',', t_23=',num2str(t23,'%10.3f'),'s , x_23=',num2str(x23,'%10.3f') 'mm, v_23=',num2str(v23,'%10.f'),'mm/s' ])
disp(['Tram 3 , MUA',', t_3 =',num2str(t3,'%10.3f'),'s , x_3 =',num2str(xi3,'%10.3f') 'mm,          a_3 =',num2str(a3,'%10.f'),'mm/s2' ])
disp(['Tram 34, MU ',', t_34=',num2str(t34,'%10.3f'),'s , x_34=',num2str(x34,'%10.3f') 'mm, v_34=',num2str(v34,'%10.f'),'mm/s' ])
disp(['Tram 4 , MUA',', t_4 =',num2str(t4,'%10.3f'),'s , x_4 =',num2str(xi4,'%10.3f') 'mm,          a_4 =',num2str(a4,'%10.f'),'mm/s2' ])

disp(' ')
disp(['temps total = =',num2str(ttotal2,'%10.3f'),'s , x total =',num2str(abs(xtotal2),'%10.3f'), 'mm'])

% Definir trams de moviment: motion control
ti(1)=      t1;
ti(2)=ti(1)+t12;
ti(3)=ti(2)+t2;
ti(4)=ti(3)+t23;
ti(5)=ti(4)+t3;
ti(6)=ti(5)+t34;
ti(7)=ti(6)+t4;

xi(1)=      xi1;
xi(2)=xi(1)+x12;
xi(3)=xi(2)+xi2;
xi(4)=xi(3)+x23;
xi(5)=xi(4)+xi3;
xi(6)=xi(5)+x34;
xi(7)=xi(6)+xi4;

% Obtenir la trajectoria posicio, velocitat i acceleració
t_ = [0:0.05:t_total];
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= ti(1)
        x_(ii) = 0 + 1/2*a1*t_(ii)^2;
        v_(ii) = a1*t_(ii);
        a_(ii) = a1;
    elseif  t_(ii) <= ti(2)
        x_(ii) = xi(1) + v12*(t_(ii)-ti(1));
        v_(ii) = v12;
        a_(ii) = 0;
    elseif    t_(ii) <= ti(3)
        x_(ii) = xi(2) + v12*(t_(ii)-ti(2)) + 1/2*a2*(t_(ii)-ti(2))^2;
        v_(ii) = v12 + a2*(t_(ii)-ti(2));
        a_(ii) = a2;
    elseif  t_(ii) <= ti(4)
        x_(ii) = xi(3) + v23*(t_(ii)-ti(3));
        v_(ii) = v23;
        a_(ii) = 0;
    elseif    t_(ii) <= ti(5)
        x_(ii) = xi(4) + v23*(t_(ii)-ti(4)) + 1/2*a3*(t_(ii)-ti(4))^2;
        v_(ii) = v23 + a3*(t_(ii)-ti(4));
        a_(ii) = a3;
    elseif  t_(ii) <= ti(6)
        x_(ii) = xi(5) + v34*(t_(ii)-ti(5));
        v_(ii) = v34;
        a_(ii) = 0;
    else
        x_(ii) = xi(6) + v34*(t_(ii)-ti(6)) + 1/2*a4*(t_(ii)-ti(6))^2;
        v_(ii) = v34 + a4*(t_(ii)-ti(6));
        a_(ii) = a4;
    end
end

% Representar poció, velocitat i acceleració
H2=figure;
subplot(3,1,1);
plot(t_,x_,'LineWidth',2)
ylabel('$u$ [mm]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{u}$ [mm/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{u} \; \rm{[mm/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
%saveas(H2,'./figures/CIN_Ex11_met2.svg')
%saveas(H2,'./figures/CIN_Ex11_met2.pdf')

% Representar omega motor
H21=figure;font_size=18;
plot(t_,v_*i/rP,'LineWidth',2)
ylabel('$\omega_m$ [rad/s]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
H21.Position = [100 100 560 240];
%saveas(H21,'./figures/CIN_Ex11_met2_omg.svg')
%saveas(H21,'./figures/CIN_Ex11_met2_omg.pdf')