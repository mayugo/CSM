#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
%% Sol·lució Exercici transportador rodets caixes
% Implemntacio model dinàmic
%  J.A.Mayugo, UdG, 3
"""

import numpy as np
import matplotlib.pyplot as plt
# import sympy as sp
from sympy.abc import s
from sympy import Poly
from sympy.physics.control.lti import TransferFunction, Feedback, Series, Parallel
from scipy import signal


# %% DADES PROBLEMA
r  =  20e-3     # m, radi politja petita
# dades motor MAXON, 20W, A-max 2019
n_max = 6460    # rpm, velocitat màxima 
n_nom = 5060    # rpm, velociat nominal
Pot_nom = 120   # W, potència nominal
V_nom = 24      # V, voltatge nominal
R = 3.99        # Ohms, resitència induït
L = 0.556e-3    # H, impedància induït
mmot = 240e-3   # kg, massa motor
Jmot = 45.3e-3*1e-4 # kgm2, inèrcia rotor 45.3 grcm2
bm = 2.8e-6;    # Ns/m, dissipació motor
# dades del reductor 
i = 168;  
mred = 190e-3   # kg, massa reductor
Jred = 0.7e-3*1e-4*i**2 # kgm2, inèrcia 0.7 grcm2
# dimensions
rR = 50e-3;
rP =  40e-3;
pitch = 125e-3;
# masses i inèrcies
mR = 6.5  # kg, massa 
NR = 11;
NC = 5;
JR = 6.2*(25e-3)**2; 
mC = 0.360 # kg, massa
c  = 0.28e-3 # Ns/m

# %% Resolució APARTAT a)
i_t = i/rR;
omega_nom = n_nom*np.pi/30
vC = rR * omega_nom/i

# %% Resolució APARTAT b)
omega_n=n_nom/30*np.pi
M_nom= Pot_nom/omega_n #%-- Nm
A_nom = (omega_n*M_nom)/V_nom 
K_m = M_nom/A_nom  # %-- Nm / A
K_b = V_nom/omega_n  #%-- V s / rad
P_nom = omega_n*M_nom

# %% Resolució APARTAT c)
J_e = Jmot + Jred/i**2 + JR*NR/i**2 + mC*NC/i_t**2
b_e = bm + c/i**2  *NR
b_e_ = b_e + (K_m*K_b)/R

# % Models
def model_2_tf(eq):
    num, den = [[float(i) for i in Poly(i, s).all_coeffs()] for i in eq.as_numer_denom()]
    return signal.lti(num, den)

# def model_2_zpk(eq):
#     num, den = [[float(i) for i in sp.Poly(i, s).all_coeffs()] for i in eq.as_numer_denom()]
#     return signal.tf2zpk(num,den)

model_omg_m  = K_m / ((J_e*s+b_e)*(L*s+R)+K_b*K_m)
model_u_c    = model_omg_m/s/i_t
tf_omg_m = model_2_tf(model_omg_m)
tf_u_c   = model_2_tf(model_u_c)

model_omg_mS = K_m / ((J_e*s+b_e)*(R)+K_b*K_m)
model_u_cS   = model_omg_mS/s/i_t
tf_omg_mS = model_2_tf(model_omg_mS)
tf_u_cS   = model_2_tf(model_u_cS)

# % Diagrama de Bode i R-Locus
wRange = np.logspace(np.log10(0.1), np.log10(1e4), num=100)
w, mag, phase = tf_omg_m.bode(w=wRange)
plt.figure()
plt.subplots(2, 1, sharex='col')
plt.subplot(2, 1, 1)
plt.semilogx(w, mag)    # Bode magnitude plot
plt.title('Bode Diagram: input: Volts, output: omega_m')
plt.ylabel('Magnitude [Db]')
plt.subplot(2, 1, 2)
plt.semilogx(w, phase)  # Bode phase plot
plt.ylabel('Phase [deg]')
plt.xlabel('Frequency [rad/s]')
plt.show()

wS, magS, phaseS = tf_u_c.bode(w=wRange)
plt.figure()
plt.subplots(2, 1, sharex='col')
plt.subplot(2, 1, 1)
plt.semilogx(wS, magS)    # Bode magnitude plot
plt.title('Bode Diagram: input: Volts, output: u')
plt.ylabel('Magnitude [Db]')
plt.subplot(2, 1, 2)
plt.semilogx(wS, phaseS)  # Bode phase plot
plt.ylabel('Phase [deg]')
plt.xlabel('Frequency [rad/s]')
plt.show()

w, H = tf_omg_m.freqresp()
plt.figure()
plt.plot(H.real, H.imag, "b")
plt.plot(H.real, -H.imag, "r")
plt.xlabel('Real Axix [1/s]')
plt.ylabel('Imaginary Axix [1/s]')
plt.title('R-Locus: input: Volts, output: omega_m')
plt.show()

wS, HS = tf_omg_mS.freqresp()
plt.figure()
plt.plot(HS.real, HS.imag, "b")
plt.plot(HS.real, -HS.imag, "r")
plt.xlabel('Real Axix [1/s]')
plt.ylabel('Imaginary Axix [1/s]')
plt.title('R-Locus: input: Volts, output: omega_m (model simplificat)')
plt.show()

# % Resposta transitòria: escaló en llaç obert:

# step(u * model_omg_m,0.1)

tRange = np.linspace(0, 0.1, num=100)

t, y = tf_omg_m.step(T=tRange)
# t, y = signal.step2(tf_omg_m,T=tRange)
plt.plot(t, y*V_nom)
plt.xlabel('t [s]')
plt.ylabel('$\omega_m$ [rad/s]')
plt.title('Resposta a una funció escaló amb ' + str(V_nom) + 'V al motor')
plt.grid()
plt.xlim([0, t[-1]])
# plt.ylim([0, 110])
plt.show()

t, y = tf_u_c.step(T=tRange)
plt.plot(t, y*V_nom)
plt.xlabel('t [s]')
plt.ylabel('desplaçament [m]')
plt.title('Resposta a una funció escaló amb ' + str(V_nom) + 'V al carro')
plt.grid()
plt.xlim([0, t[-1]])
# plt.ylim([0, 110])
plt.show()


# %% Resolució APARTAT d)
# % Matrius de massa M_, esmorteïment C_ i rigidesa K_
f_v  = np.array([K_m/R])    # % adaptar inputs en voltatge
M_ = np.array([J_e])   
C_ = np.array([b_e_])
K_ = np.array([0])

# % Definició de l'espai d'estat
n_gdl = M_.shape[0]
# A = [ zeros(n_gdl) , eye(n_gdl);  -inv(M_)*K_   , -inv(M_)*C_]
# B = [ zeros(n_gdl) ; inv(M_)]*f_v       % inputs voltatges motors
# C = [ 0  1 ; 1/i_t  0];       % defineix output velocitats angulars
# D = [ 0 ; 0];
# model = ss(A, B, C, D,  'statename', {'\theta_{motor}' 'omg_motor'},...
#                         'inputname', {'U_{motor} (V)'},...
#                         'outputname',{'omg_{motor} (rad/s)' 'desp_{caixa} (m/s)'})

# % Diagrama de Bode i R-Locus
# h=figure;
# h1 = bodeplot(model); %,{0.1,20});%setoptions(h,'MagUnits','abs','FreqScale','linear')
# set(h,'Units','Inches');pos = get(h,'Position');
# set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
# print(h,'2021ss_bode','-dpdf','-r0')

# %% Resposta transitòria
# % escaló en llaç obert:
# u = V_nom;  %Volts al motor 
# h=figure;
# step(u * model,0.1)
# grid; %title(['Resposta a una funció escaló amb ' num2str(u) ' V al motor'])
# set(h,'Units','Inches');pos = get(h,'Position');
# set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
# print(h,'2021ss_resposta','-dpdf','-r0')

# %% Conversió d'espai d'estat H_s = C*inv(s*I-A)*B+D
# [num,den] = ss2tf(A,B,C,D,1)  
# H_s11 = tf(num(1,:),den);H_s11=minreal(H_s11)

# %FT1= H_s11*R
# %fig1=figure;bode(H_s11,model_omg_mS);%legend('H_{s11}','H_{s22}')
# %fig2=figure;rlocus(H_s11);%title('Root Locus H_{s11}')


