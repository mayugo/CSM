%% Sol·lució implementada trajectòria posicionador safates autoclau
% Generació de trajectòries, Exemple 1.2
% 01_cin_01.tex
% J.A.Mayugo, UdG, 2023

close all; clear;
%% DADES
% Dimensions
l_e = 150;      % mm, distància entre autcolaus
l_t = 900;      % mm, distància total moviment l_e x 6 
l_eix = 1000;   % mm, separació entre eixos eixos politges

r = 40;         % mm, radi primitiu politja
i = 36;         %   , relació reductor moto-reductor

% Temps per la trajectoria
t0 = 1;         % s, temps parat entre moviments

t1 = 3.0;       % s, temps total primer moviment (IN a A5), carregat
t2 = 2.0;       % s, temps total segon moviment (A5 a A1), descarregat
t3 = 2.5;       % s, temps total tercer moviment (A1 a OUT), carregat
t4 = 3.0;       % s, temps total quart moviment (OUT a IN), descarregat

a = 1000;   % mm/s2, accelarció màxima 

%% Resolució: marxa-moviment-parada
t_inc = 0.01;    % s, increment de temps per la simulació moviment

% desplaçaments a cada moviment 
x1 =  l_e*5;
x2 =  (x1-l_e);
x3 =  l_e*5;
x4 =  l_t;       % també com l_t = l_e*6

% parametres dels 4 trams
[tb1,xb1,tc1,xc1,v1]=parametres_cicle(t1,x1,a);
[tb2,xb2,tc2,xc2,v2]=parametres_cicle(t2,x2,a);
[tb3,xb3,tc3,xc3,v3]=parametres_cicle(t3,x3,a);
[tb4,xb4,tc4,xc4,v4]=parametres_cicle(t4,x4,a);

% En el motor:
omega_max       = max(abs([v1,v2,v3,v4]))*i/r;
n_max           = omega_max*60/(2*pi);

eps_max       = a*i/r;

% Definir trams de moviment: comptant parades 14 trams
ti(1) =         tb1;
ti(2) = ti(1) + tc1;
ti(3) = ti(2) + tb1;
ti(4) = ti(3) + t0;
ti(5) = ti(4) + tb2;
ti(6) = ti(5) + tc2;
ti(7) = ti(6) + tb2;
ti(8) = ti(7) + t0;
ti(9) = ti(8) + tb3;
ti(10) = ti(9) + tc3;
ti(11) = ti(10) + tb3;
ti(12) = ti(11) + tb4;
ti(13) = ti(12) + tc4;
ti(14) = ti(13) + tb4;

t_total = ti(14)

xi(1) =                             + 1/2*a*ti(1)^2;
xi(2) =  xi(1)  + v1*(ti(2)-ti(1));
xi(3) =  xi(2)  + v1*(ti(3)-ti(2))  - 1/2*a*(ti(3)-ti(2))^2;
xi(4) =  xi(3);
xi(5) =  xi(4)                      - 1/2*a*(ti(5)-ti(4))^2;
xi(6) =  xi(5)  - v2*(ti(6)-ti(5));
xi(7) =  xi(6)  - v2*(ti(7)-ti(6))  + 1/2*a*(ti(7)-ti(6))^2;
xi(8) =  xi(7);
xi(9) =  xi(8)                      + 1/2*a*(ti(9)-ti(8))^2;
xi(10) = xi(9)  + v3*(ti(10)-ti(9));
xi(11) = xi(10) + v3*(ti(11)-ti(10))- 1/2*a*(ti(11)-ti(10))^2;
xi(12) = xi(11)                     - 1/2*a*(ti(12)-ti(11))^2;
xi(13) = xi(12) - v4*(ti(13)-ti(12));
xi(14) = xi(13) - v4*(ti(14)-ti(13))+ 1/2*a*(ti(14)-ti(13))^2;

x_total = xi(14)

vi = [ v1, v1, 0, 0, -v2, -v2, 0, 0, v3, v3, 0, -v4, -v4, 0];
ai = [ a, -a, -a, -a, -a, a, a, a, a, -a, -a, -a, a, a];

% Obtenir la trajectoria posicio, velocitat i acceleració
t_ = [0:t_inc:t_total];
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= ti(1)
        x_(ii) = + 1/2*a*t_(ii)^2;
        v_(ii) = a*t_(ii);
        a_(ii) = a;
    elseif  t_(ii) <= ti(2)
        x_(ii) = xi(1) + v1*(t_(ii)-ti(1));
        v_(ii) = v1;
        a_(ii) = 0;
    elseif  t_(ii) <= ti(3)
        x_(ii) = xi(2) + v1*(t_(ii)-ti(2)) - 1/2*a*(t_(ii)-ti(2))^2;
        v_(ii) =  v1 - a*(t_(ii)-ti(2));
        a_(ii) = - a;
    elseif  t_(ii) <= ti(4)
        x_(ii) = xi(3);
        v_(ii) = 0;
        a_(ii) = 0;
    elseif  t_(ii) <= ti(5)
        x_(ii) = xi(4) - 1/2*a*(t_(ii)-ti(4))^2;
        v_(ii) = - a*(t_(ii)-ti(4));
        a_(ii) = - a;
    elseif  t_(ii) <= ti(6)
        x_(ii) = xi(5) - v2*(t_(ii)-ti(5));
        v_(ii) = -v2;
        a_(ii) = 0;
    elseif  t_(ii) <= ti(7)
        x_(ii) = xi(6) - v2*(t_(ii)-ti(6)) + 1/2*a*(t_(ii)-ti(6))^2;
        v_(ii) = -v2 + a*(t_(ii)-ti(6));
        a_(ii) = a;
    elseif  t_(ii) <= ti(8)
        x_(ii) = xi(7);
        v_(ii) = 0;
        a_(ii) = 0;
    elseif   t_(ii) <= ti(9)
        x_(ii) = xi(8) + 1/2*a*(t_(ii)-ti(8))^2;
        v_(ii) = a*(t_(ii)-ti(8));
        a_(ii) = a;
    elseif  t_(ii) <= ti(10)
        x_(ii) = xi(9) + v3*(t_(ii)-ti(9));
        v_(ii) = v3;
        a_(ii) = 0;
    elseif  t_(ii) <= ti(11)
        x_(ii) = xi(10) + v3*(t_(ii)-ti(10)) - 1/2*a*(t_(ii)-ti(10))^2;
        v_(ii) =  v3 - a*(t_(ii)-ti(10));
        a_(ii) = -a;
    elseif  t_(ii) <= ti(12)
        x_(ii) = xi(11) - 1/2*a*(t_(ii)-ti(11))^2;
        v_(ii) = -a*(t_(ii)-ti(11));
        a_(ii) = -a;
    elseif  t_(ii) <= ti(13)
        x_(ii) = xi(12) - v4*(t_(ii)-ti(12));
        v_(ii) = -v4;
        a_(ii) = 0;
    else
        x_(ii) = xi(13) -v4*(t_(ii)-ti(13)) + 1/2*a*(t_(ii)-ti(13))^2;
        v_(ii) = -v4 +a*(t_(ii)-ti(13));
        a_(ii) = a;
    end
end

% Representar poció, velocitat i acceleració
H1=figure;font_size=18;
subplot(3,1,1);hold on;
plot(t_,x_,'LineWidth',2)
plot([0 ti],[0 xi],'o','LineWidth',2)
ylabel('${u}$ [mm]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,2);hold on;
plot(t_,v_,'LineWidth',2)
plot([0 ti],[0 vi],'+','LineWidth',2)
ylabel('$\dot{u}$ [mm/s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(3,1,3);hold on;
plot(t_,a_,'LineWidth',2)
plot([0 ti],[a ai],'x','LineWidth',2)
ylabel('$\ddot{u} \; \rm{[mm/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H1,'./figures/CSM_autoclau_01_cin_01a.svg')
saveas(H1,'./figures/CSM_autoclau_01_cin_01a.pdf')

% Representar omega motor
H2=figure;font_size=18;hold on;
subplot(2,1,1);hold on;
plot(t_,v_*i/r*(30/pi),'LineWidth',2)
plot([0 ti],[0 vi]*i/r*(30/pi),'+','LineWidth',2)
ylabel('$n_m$ [rpm]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
subplot(2,1,2);hold on;
plot(t_,a_*i/r,'LineWidth',2)
plot([0 ti],[a ai]*i/r,'x','LineWidth',2)
ylabel('$\varepsilon_m$ [rad/s$^2$]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_total])
saveas(H2,'./figures/CSM_autoclau_01_cin_01b.svg')
saveas(H2,'./figures/CSM_autoclau_01_cin_01b.pdf')

%% Funcions

function [tb,xb,tc,xc,v_max]=parametres_cicle(t,x,a_max)
% Input:  temps total (t), distància total (x), acceleració màxima (a_max)
% Output: temps arrancada/parada (tb), temps marxa vel. constant (tc)
%         ditància arrancada/parada (tb), distància marxa vel. constant (tc)
    tb = t/2 - (a_max^2*t^2-4*a_max*x)^0.5/(2*a_max);  % temps arrancada o parada
    xb = 1/2*a_max*tb^2;         % distància arrancada o parada
    tc = t - 2*tb;               % temps marxa velocitat constant
    xc = x - 2*xb;               % distància marxa velocitat constant
    v_max = a_max*tb;            % velocitat màxima
end

