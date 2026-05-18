import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt

# ============================================================
# MODEL COMPLET DEL MOTOR DC
# ============================================================

# ---------- Paràmetres ----------
Ra = 0.3                 # Ohm
La = 15e-3               # H

J  = 671.97e-6           # kg m^2
b  = 18.34e-6            # N m s/rad

Km = 0.152
Kb = 0.152

# Conversió final (si la vols mantenir)
Kconv = 0.05 / 30

# ============================================================
# G(s) = theta(s)/U(s)
#
#                 Km
# --------------------------------- * 1/s * Kconv
# (La s + Ra)(J s + b) + Kb Km
# ============================================================

# ------------------------------------------------------------
# DENOMINADOR MECÀNIC + ELÈCTRIC
# ------------------------------------------------------------

# (La s + Ra)(J s + b)
den1 = np.polymul([La, Ra], [J, b])

# + Kb*Km
den1[-1] += Kb * Km

# multiplicar per s  -> posició angular
den_full = np.polymul(den1, [1, 0])

# numerador
num_full = [Km * Kconv]

print("\n==============================")
print("MODEL COMPLET")
print("==============================")
print("Numerador:")
print(num_full)

print("\nDenominador:")
print(den_full)

# Sistema complet
G_full = sig.TransferFunction(num_full, den_full)

# ============================================================
# MODEL SIMPLIFICAT
# ============================================================

# tau_e = L/R
tau_e = La / Ra

# tau_m aproximada
tau_m = J / b

print("\n==============================")
print("CONSTANTS DE TEMPS")
print("==============================")
print(f"tau_e = {tau_e:.4f} s")
print(f"tau_m = {tau_m:.4f} s")

# ------------------------------------------------------------
# Aproximació:
#
# si tau_e << tau_m
# ignorem la dinàmica elèctrica
#
# Gs(s) ≈ K / ( s (tau_m s + 1) )
# ------------------------------------------------------------

# guany aproximat
K_approx = (Km / (Ra * b + Kb * Km)) * Kconv

num_simp = [K_approx]

den_simp = [tau_m, 1, 0]

G_simp = sig.TransferFunction(num_simp, den_simp)

print("\n==============================")
print("MODEL SIMPLIFICAT")
print("==============================")
print("Numerador:")
print(num_simp)

print("\nDenominador:")
print(den_simp)

# ============================================================
# RESPOSTA A ESGLAÓ EN LLAÇ OBERT
# ============================================================

t = np.linspace(0, 60, 3000)

t1, y1 = sig.step(G_full, T=t)
t2, y2 = sig.step(G_simp, T=t)

plt.figure(figsize=(10,5))

plt.plot(t1, y1, label='Model complet')
plt.plot(t2, y2, '--', label='Model simplificat')

plt.xlabel('Temps [s]')
plt.ylabel('Angle')
plt.title('Resposta a esglaó - llaç obert')
plt.grid(True)
plt.legend()

# ============================================================
# BODE
# ============================================================

w = np.logspace(-1, 3, 2000)

w1, mag1, phase1 = sig.bode(G_full, w=w)
w2, mag2, phase2 = sig.bode(G_simp, w=w)

plt.figure(figsize=(10,5))

plt.semilogx(w1, mag1, label='Complet')
plt.semilogx(w2, mag2, '--', label='Simplificat')

plt.xlabel('Freq [rad/s]')
plt.ylabel('Magnitud [dB]')
plt.title('Bode Magnitud')
plt.grid(True, which='both')
plt.legend()

plt.figure(figsize=(10,5))

plt.semilogx(w1, phase1, label='Complet')
plt.semilogx(w2, phase2, '--', label='Simplificat')

plt.xlabel('Freq [rad/s]')
plt.ylabel('Fase [deg]')
plt.title('Bode Fase')
plt.grid(True, which='both')
plt.legend()

# ============================================================
# POLS
# ============================================================

poles_full = np.roots(den_full)
poles_simp = np.roots(den_simp)

print("\n==============================")
print("POLS MODEL COMPLET")
print("==============================")
for p in poles_full:
    print(p)

print("\n==============================")
print("POLS MODEL SIMPLIFICAT")
print("==============================")
for p in poles_simp:
    print(p)

# ============================================================
# PROPOSTA PID
# ============================================================

# Idea:
#
# triem wc aproximadament
# 5-10 vegades més petit que el pol ràpid
#

real_poles = np.real(poles_full)

fastest_pole = np.max(np.abs(real_poles))

wc = fastest_pole / 8

print("\n==============================")
print("ESTIMACIÓ PID")
print("==============================")
print(f"Pol ràpid ~ {fastest_pole:.2f} rad/s")
print(f"wc triada ~ {wc:.2f} rad/s")

# ------------------------------------------------------------
# PI inicial
# ------------------------------------------------------------

# aproximació:
#
# Kp ~ wc / K
# Ki ~ wc^2 / 5
#

Kp = wc / K_approx
Ki = wc**2 / 5

# petit derivatiu opcional
Kd = Kp / (10 * wc)

print(f"\nKp = {Kp:.3f}")
print(f"Ki = {Ki:.3f}")
print(f"Kd = {Kd:.5f}")

# ============================================================
# TANCAMENT PID
# ============================================================

# PID:
#
# C(s) = Kd s^2 + Kp s + Ki
#        -------------------
#                 s
#

num_pid = [Kd, Kp, Ki]
den_pid = [1, 0]

# L(s) = C(s)G(s)
num_ol = np.polymul(num_pid, num_full)
den_ol = np.polymul(den_pid, den_full)

# T(s) = L/(1+L)

# denominador tancat:
# den_cl = den_ol + num_ol

# igualar dimensions
if len(den_ol) > len(num_ol):
    num_ol = np.pad(num_ol,
                    (len(den_ol)-len(num_ol), 0))
else:
    den_ol = np.pad(den_ol,
                    (len(num_ol)-len(den_ol), 0))

den_cl = den_ol + num_ol
num_cl = num_ol

G_cl = sig.TransferFunction(num_cl, den_cl)

# resposta tancada
t3, y3 = sig.step(G_cl, T=t)

plt.figure(figsize=(10,5))

plt.plot(t3, y3)

plt.xlabel('Temps [s]')
plt.ylabel('Angle')
plt.title('Resposta tancada amb PID')
plt.grid(True)



 # ------------------------------------------------
# MODEL VELOCITAT
# ------------------------------------------------

den_speed = den1
num_speed = [Km * Kconv]

G_speed = sig.TransferFunction(num_speed, den_speed)

t, y = sig.step(G_speed, T=np.linspace(0,1,3000))

plt.figure(figsize=(10,5))
plt.plot(t,y)

plt.title("Resposta velocitat")
plt.grid(True)

plt.show()
