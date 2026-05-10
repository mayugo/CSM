import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# =========================
# 1. SISTEMA
# =========================

# Gm(s) = (s+1)/(s+3)
numG = [1, 1]
denG = [1, 3]

G_m = signal.lti(numG, denG)

t = np.arange(0, 7, 0.001)

U = 1
u = U * np.ones_like(t)

# resposta llaç obert
t_ol, y_ol, _ = signal.lsim(G_m, U=u, T=t)

plt.figure()
plt.plot(t_ol, y_ol, linewidth=1.8)
plt.grid()
plt.title('Resposta a escaló unitari')
plt.xlabel('t [s]')
plt.ylabel('Resposta [eu]')
plt.show()

# =========================
# 2. PID
# =========================

err_v = 0.5

Ki = 3 / err_v
Kp = 94 / 65
Kd = 43 / 65

# PID(s) = (Kd s^2 + Kp s + Ki)/s
numC = [Kd, Kp, Ki]
denC = [1, 0]

# =========================
# 3. LLAÇ TANCAT
# =========================

# open loop
num_ol = np.polymul(numC, numG)
den_ol = np.polymul(denC, denG)

# closed loop: T = N / (D + N)
num_cl = num_ol
den_cl = np.polyadd(den_ol, num_ol)

G_cl = signal.lti(num_cl, den_cl)

poles = np.roots(den_cl)
print("Pols llaç tancat:", poles)

# =========================
# 4. RESPOSTA POSICIÓ
# =========================

t_out, pos, _ = signal.lsim(G_cl, U=u, T=t)

# =========================
# 5. VELOCITAT (G/s)
# =========================

num_vel = num_cl
den_vel = np.polymul(den_cl, [1, 0])

G_vel = signal.lti(num_vel, den_vel)

t_out, vel, _ = signal.lsim(G_vel, U=u, T=t)

# =========================
# 6. PLOTS
# =========================

fig, axs = plt.subplots(2, 1, figsize=(7, 6), sharex=True)

# posició
axs[0].plot(t, u, linewidth=1.2, label='consigna')
axs[0].plot(t_out, pos, linewidth=1.8, label='resposta')
axs[0].set_ylabel('consigna [eu]')
axs[0].grid()
axs[0].legend()

# velocitat
axs[1].plot(t, t, linewidth=1.2, label='referència')
axs[1].plot(t_out, vel, linewidth=1.8, label='resposta')
axs[1].set_xlabel('t [s]')
axs[1].set_ylabel('velocitat')
axs[1].grid()
axs[1].legend()

plt.tight_layout()
plt.show()