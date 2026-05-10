%%% Exemple sintonització ZN
% Implementar PID a un sistema de 3r ordre
% J.A.Mayugo, UdG, 2023

clear; close all;

%% Propietats del sistema
s   = tf('s');
G_m = 6/(48*s^3+44*s^2+12*s+1);

%% APARTAT: Sintonització en llaç obert
% Resposta transitòria en llaç obert:
t_inc = 0.1;t_f=60;
t_ = 0:t_inc:t_f;    
U_ = ones(size(t_));
[Y,T_,X] = lsim(G_m,U_,t_);

[DeltaY,I] = max(diff(Y)/t_inc);            % pendent màxime, inflexió
out = [0, Y(I), 6]; % punt inferior, punt d'inflexió corba S, punt superior
t = zeros(1,3);
t(1)= t_(I) - (out(2))/DeltaY;           % punt inferior de la tangent
t(2)= t_(I);                             % punt d'inflexió de la corba S
t(3)= t_(I) + (out(3)-out(2))/DeltaY;    % punt superior de la tangent

H_ol=figure;hold on
plot(t_,Y  ,'LineWidth',1.8)
plot(t ,out,'--o','LineWidth',1.4)
%plot(t_(:,1:length(diff(Y))),diff(Y)) % derivada
grid; title(['Resposta a una funció escaló unitària llaç obert'],'FontSize',22)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);ylabel('Resposta [eu]','Interpreter','latex','FontSize',24)
text(t(1),out(1),{" (L,0)"},'FontSize',18);  % ' first line' \newline 
text(t(2),out(2),{" Punt d'inflexió"},'FontSize',18);
text(t(3),out(3),{" (L+T,est.)"},'FontSize',18);
set(gca,'FontSize',18);
saveas(H_ol,'./figures/Exemple_4_PID_3r_ordre_ZN.svg')
saveas(H_ol,'./figures/Exemple_4_PID_3r_ordre_ZN.pdf')

% FUNCIÓ AMB L'ORDRE BODE de MatLab, R-Locus, i pols
H_bode=figure;
h = bodeplot(G_m); %,{0.1,20});
H_rlocus=figure;
rlocus(G_m);
p = pole(G_m)

% Sintonització llaç tancat controlador PID
K = round(max(Y),2);
L = t(1);        
T = t(3)-t(1); 

legend_=["Ziegler-Nichols OL classic",
         "Ziegler-Nichols OL some overshoot",
         "Ziegler-Nichols OL no overshoot",
         "Manual ($K_p=0.6KT/L$, $T_i=4L$, $T_d=L$)"
         ];
Kp_ = [1.2*T/L, 1.2*T/L*0.6/0.33,  1.2*T/L/3, 1.2*T/L/2]/K;
Ti_ = [2*L,     2*L,               2*L,       2*L*2];
Td_ = [0.5*L,   0.5*L*8/3,         0.5*L*8/3,    0.5*L*2];

u_ = ones(size(t_));
for ii = 1:length(Kp_)
    Kp = Kp_(ii);
    Ti = Ti_(ii);
    Td = Td_(ii);
    
    PID  = Kp*(1+1/(Ti*s)+Td*s); % controlador PID
    G_cl = PID*G_m/(1+PID*G_m);% model llaç tancat amb PI, fórmula de Mason
    
    [omg_,T_,X] = lsim(G_cl,u_,t_);  
    OMG_(ii,:)=omg_;
end

H1_cl=figure;
hold on
for ii = 1:length(Kp_)
    plot(t_,OMG_(ii,:),'LineWidth',1.8);
end
title(['Resposta control PID, consigna escaló unitari'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('Resposta [eu]','Interpreter','latex','FontSize',24);
legend(legend_,'Interpreter','latex','FontSize',18,'Location','south');
set(gca,'FontSize',18); legend boxoff; grid;
saveas(H1_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_1.svg')
saveas(H1_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_1.pdf')

OMG_=[];
tfinal = 300;
t_ = 0:0.1:tfinal;    % resposta temporal
u_ = ones(size(t_));

H2_cl=figure;
subplot(2,3,1)
Kcr = 1.0;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
subplot(2,3,2)
Kcr = 2.0;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
subplot(2,3,3)
Kcr = 1.5;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
subplot(2,3,4)
Kcr = 1.6;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
subplot(2,3,5)
Kcr = 1.65;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
subplot(2,3,6)
Kcr = 1.666;
Gcr_cl = Kcr*G_m/(1+Kcr*G_m);% model llaç amb P, fórmula de Mason
[omg_,T_,X] = lsim(Gcr_cl,u_,t_);
plot(t_,omg_,'LineWidth',1.8);axis off
legend(["K_{cr} = "+num2str(Kcr)],'FontSize',14,'Location','northoutside');
legend boxoff;set(gca,'FontSize',14);
saveas(H2_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_2.svg')
saveas(H2_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_2.pdf')

Pcr = tfinal/sum(islocalmax(omg_));

legend_=["Ziegler-Nichols CL classic",
         "Pessen Integral Rule CL",
         "Ziegler-Nichols CL some overshoot",
         "Ziegler-Nichols CL no overshoot"];
Kp_ = [0.6*Kcr 0.7*Kcr   0.33*Kcr 0.2*Kcr ];
Ti_ = [Pcr/2   Pcr/2.5   Pcr/2    Pcr/2   ];
Td_ = [Pcr/8   0.15*Pcr  Pcr/3    Pcr/3   ];

OMG_=[];
t_ = 0:0.1:60;    % resposta temporal

u_ = ones(size(t_));
for ii = 1:length(Kp_)
    Kp = Kp_(ii);
    Ti = Ti_(ii);
    Td = Td_(ii);
    
    PID  = Kp*(1+1/(Ti*s)+Td*s); % controlador PID
    G_cl = PID*G_m/(1+PID*G_m);% model llaç tancat amb PID, fórmula de Mason
    
    [omg_,T_,X] = lsim(G_cl,u_,t_);  
    OMG_(ii,:)=omg_;
end

H3_cl=figure;
hold on
for ii = 1:length(Kp_)
    plot(t_,OMG_(ii,:),'LineWidth',1.8);
end
title(['Resposta control PID, consigna escaló unitari'],'FontSize',24)
xlabel('$t$ [s]','Interpreter','latex','FontSize',24);
ylabel('Resposta [eu]','Interpreter','latex','FontSize',24);
legend(legend_,'Interpreter','latex','FontSize',18,'Location','south');
set(gca,'FontSize',18); legend boxoff; grid;
saveas(H3_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_3.svg')
saveas(H3_cl,'./figures/Exemple_4_PID_3r_ordre_ZN_3.pdf')