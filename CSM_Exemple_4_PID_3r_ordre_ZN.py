# %%% Exemple sintonització ZN
# % Implementar PID a un sistema de 3r ordre
# % J.A.Mayugo, UdG, 2023

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# =========================
# 1. SISTEMA
# =========================

# Gm(s) = 6 / (48 s^3 + 44 s^2 + 12 s + 1)
numG = [6]
denG = [48, 44, 12, 1]

G_m = signal.lti(numG, denG)

t = np.arange(0, 60, 0.1)
u = np.ones_like(t)

# resposta open loop
t_ol, y_ol, _ = signal.lsim(G_m, U=u, T=t)

plt.figure()
plt.plot(t_ol, y_ol, linewidth=1.8)
plt.grid()
plt.title("Resposta escaló llaç obert")
plt.xlabel("t [s]")
plt.ylabel("Resposta [eu]")
plt.show()

# =========================
# 2. BODE
# =========================

w = np.logspace(-3, 2, 600)
w, H = signal.freqresp((numG, denG), w)

plt.figure()
plt.semilogx(w, 20*np.log10(np.abs(H)))
plt.grid()
plt.title("Bode Magnitud")
plt.xlabel("ω")
plt.ylabel("dB")
plt.show()

# =========================
# 3. ROOT LOCUS (aprox)
# =========================

K_values = np.linspace(0, 3, 120)
poles_list = []

for Kk in K_values:
    numK = np.polymul([Kk], numG)
    den_cl = np.polyadd(denG, numK)
    poles_list.append(np.roots(den_cl))

plt.figure()
for p in poles_list:
    plt.plot(np.real(p), np.imag(p), 'b.', markersize=2)

plt.grid()
plt.title("Root locus (aprox)")
plt.xlabel("Real")
plt.ylabel("Imag")
plt.show()

# =========================
# 4. IDENTIFICACIÓ ZN (S CURVE)
# =========================

dY = np.diff(y_ol) / 0.1
I = np.argmax(dY)

DeltaY = dY[I]

out = [0, y_ol[I], np.max(y_ol)]

t_pts = np.zeros(3)
t_pts[0] = t[I] - out[1] / DeltaY
t_pts[1] = t[I]
t_pts[2] = t[I] + (out[2] - out[1]) / DeltaY

L = t_pts[0]
T = t_pts[2] - t_pts[0]
Kstatic = np.max(y_ol)

# =========================
# 5. PID ZN (OPEN LOOP DESIGN)
# =========================

Kp_ = []
Ti_ = []
Td_ = []

Kp_.append(1.2 * T / L / Kstatic)
Ti_.append(2 * L)
Td_.append(0.5 * L)

Kp_.append((1.2 * T / L * 0.6 / 0.33) / Kstatic)
Ti_.append(2 * L)
Td_.append(0.5 * L * 8/3)

Kp_.append((1.2 * T / L / 3) / Kstatic)
Ti_.append(2 * L)
Td_.append(0.5 * L * 8/3)

Kp_.append((1.2 * T / L / 2) / Kstatic)
Ti_.append(2 * L * 2)
Td_.append(0.5 * L * 2)

# =========================
# 6. SIMULACIÓ PID (CL)
# =========================

responses = []

for Kp, Ti, Td in zip(Kp_, Ti_, Td_):

    numC = [Kp*Td, Kp, Kp/Ti]
    denC = [1, 0]

    num_ol = np.polymul(numC, numG)
    den_ol = np.polymul(denC, denG)

    num_cl = num_ol
    den_cl = np.polyadd(den_ol, num_ol)

    sys_cl = signal.lti(num_cl, den_cl)

    t_out, y, _ = signal.lsim(sys_cl, U=u, T=t)

    responses.append(y)

# =========================
# 7. PLOT PID ZN (OL)
# =========================

plt.figure()

for y in responses:
    plt.plot(t, y, linewidth=1.8)

plt.grid()
plt.title("PID Ziegler-Nichols (comparació)")
plt.xlabel("t [s]")
plt.ylabel("Resposta [eu]")
plt.show()

# =========================
# 8. PUNTS KCR (OSIL·LACIÓ)
# =========================

t_long = np.arange(0, 300, 0.1)
u_long = np.ones_like(t_long)

Kcr_values = [1.0, 2.0, 1.5, 1.6, 1.65, 1.666]

plt.figure()

for i, Kcr in enumerate(Kcr_values):

    num_cl = np.polymul([Kcr], numG)
    den_cl = np.polyadd(denG, np.polymul([Kcr], numG))

    sys = signal.lti(num_cl, den_cl)
    t_out, y, _ = signal.lsim(sys, U=u_long, T=t_long)

    plt.subplot(2, 3, i+1)
    plt.plot(t_out, y)
    plt.title(f"Kcr={Kcr}")
    plt.axis("off")

plt.tight_layout()
plt.show()

# =========================
# 9. PID FINAL ZN (CL RULES)
# =========================

Pcr = t_long[np.argmax(np.diff(Kcr_values))] if len(Kcr_values) > 0 else 1

print('Pcr:', Pcr)

Kp_final = [0.6, 0.7, 0.33, 0.2]
Ti_final = [Pcr/2, Pcr/2.5, Pcr/2, Pcr/2]
Td_final = [Pcr/8, 0.15*Pcr, Pcr/3, Pcr/3]

responses2 = []

for Kp, Ti, Td in zip(Kp_final, Ti_final, Td_final):

    numC = [Kp*Td, Kp, Kp/Ti]
    denC = [1, 0]

    num_ol = np.polymul(numC, numG)
    den_ol = np.polymul(denC, denG)

    num_cl = num_ol
    den_cl = np.polyadd(den_ol, num_ol)

    sys_cl = signal.lti(num_cl, den_cl)

    t_out, y, _ = signal.lsim(sys_cl, U=np.ones_like(t), T=t)

    responses2.append(y)

plt.figure()

for y in responses2:
    plt.plot(t, y, linewidth=1.8)

plt.grid()
plt.title("PID ZN final")
plt.xlabel("t [s]")
plt.ylabel("Resposta")
plt.show()