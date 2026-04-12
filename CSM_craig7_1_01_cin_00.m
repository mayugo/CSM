%% Sol·lució implementada Craig_7_1
% Generació de trajectòries, Exemple 1.1
% J.A.Mayugo, UdG, 2021

close all; clear;
%% DADES
t_f = 3;   % segons

theta_0 = 15; % degrees	
theta_f = 75; % degrees

% Trapezoïdal
a_max = 40;  % degrees/s2

%% Resolució: polinomi cúbic
theta = theta_f-theta_0;

a0 = theta_0;
a1 = 0;
a2 = 3/t_f^2*(theta);	
a3 = -2/t_f^3*(theta);

% Obtenir la trajectoria posicio, velocitat i acceleració
t_ = [0:1/40:t_f];
x_ = a0 + a2*t_.^2 + a3*t_.^3;
v_ = 2*a2*t_ + 3*a3*t_.^2 ;
a_ = 2*a2 + 6*a3*t_;

% Representar poció, velocitat i acceleració
H1=figure;font_size=18;
H1.Position = [10 10 900 300]; 
subplot(1,3,1);
plot(t_,x_,'LineWidth',2)
ylabel('${\theta}$ [graus]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{\theta}$ [graus/s]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{\theta} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
saveas(H1,'./figures/CIN_Ex_craig_7_1a.svg')
saveas(H1,'./figures/CIN_Ex_craig_7_1a.pdf')


%% Resolució: trapezoïdal
% Paràmetres cicle
t_b    = t_f/2 - sqrt(a_max^2*t_f^2-4*a_max*(theta))/2/a_max;
x_b    = 1/2*a_max*t_b^2;

v_max   = t_b*a_max;

% Definir trams de moviment: motion control
ti(1) = t_b;
ti(2) = ti(1) + t_f-2*t_b;
ti(3) = ti(2) + t_b;
t_total_test = sum(ti)

xi(1) = theta_0 + 1/2*a_max*ti(1)^2;
xi(2) = xi(1) + v_max*(ti(2)-ti(1));
xi(3) = xi(2) + v_max*(ti(3)-ti(2)) - 1/2*a_max*(ti(3)-ti(2))^2;
x_total_test = sum(xi)

% Obtenir la trajectoria posicio, velocitat i acceleració
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= ti(1)
        x_(ii) = theta_0 + 1/2*a_max*t_(ii)^2;
        v_(ii) = a_max*t_(ii);
        a_(ii) = a_max;
    elseif  t_(ii) <= ti(2)
        x_(ii) = xi(1) + v_max*(t_(ii)-ti(1));
        v_(ii) = v_max;
        a_(ii) = 0;
    else %if  t_(ii) <= ti(3)
        x_(ii) = xi(2) + v_max*(t_(ii)-ti(2)) - 1/2*a_max*(t_(ii)-ti(2))^2;
        v_(ii) = v_max - a_max*(t_(ii)-ti(2));
        a_(ii) = -a_max;
    end
end

% Representar poció, velocitat i acceleració
H2=figure;font_size=18;
H2.Position = [10 10 900 300]; 
subplot(1,3,1);
plot(t_,x_,'LineWidth',2)
ylabel('${\theta}$ [graus]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{\theta}$ [graus/s]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{\theta} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
saveas(H2,'./figures/CIN_Ex_craig_7_1b.svg')
saveas(H2,'./figures/CIN_Ex_craig_7_1b.pdf')


%% Resolució: triangular
a_max = 4*theta/t_f^2;

% Paràmetres cicle
t_b    = t_f/2;
x_b    = 1/2*a_max*t_b^2;

v_max   = t_b*a_max;

% Definir trams de moviment: motion control
ti(1) = t_b;
ti(2) = ti(1) + t_b;
t_total_test = sum(ti)

xi(1) = theta_0 + 1/2*a_max*ti(1)^2;
xi(2) = xi(1) + v_max*(ti(2)-ti(1)) - 1/2*a_max*(ti(2)-ti(1))^2;
x_total_test = sum(xi)

% Obtenir la trajectoria posicio, velocitat i acceleració
x_ = zeros(1,length(t_));
v_ = zeros(1,length(t_));
a_ = zeros(1,length(t_));

for ii = [1:length(t_)]
    if  t_(ii) <= ti(1)
        x_(ii) = theta_0 + 1/2*a_max*t_(ii)^2;
        v_(ii) = a_max*t_(ii);
        a_(ii) = a_max;
    else
        x_(ii) = xi(1) + v_max*(t_(ii)-ti(1)) - 1/2*a_max*(t_(ii)-ti(1))^2;
        v_(ii) = v_max - a_max*(t_(ii)-ti(1));
        a_(ii) = -a_max;
    end
end

% Representar poció, velocitat i acceleració
H3=figure;font_size=18;
H3.Position = [10 10 900 300]; 
subplot(1,3,1);
plot(t_,x_,'LineWidth',2)
ylabel('${\theta}$ [graus]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,2);
plot(t_,v_,'LineWidth',2)
ylabel('$\dot{\theta}$ [graus/s]','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
subplot(1,3,3);
plot(t_,a_,'LineWidth',2)
ylabel('$\ddot{\theta} \; \rm{[graus/s^2]}$','Interpreter','latex','FontSize',font_size)
xlabel('$t$ [s]','Interpreter','latex','FontSize',font_size)
set(gca,'FontSize',font_size)
xlim([0 t_f])
saveas(H3,'./figures/CIN_Ex_craig_7_1c.svg')
saveas(H3,'./figures/CIN_Ex_craig_7_1c.pdf')

