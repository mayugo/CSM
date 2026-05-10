import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# =========================
# 1. SISTEMA I RESPOSTA OL
# =========================

U = 1
t = np.arange(0, 5, 0.01)

# Gm(s) = 1/(s - 1)
numG = [1]
denG = [1, -1]
G_m = signal.lti(numG, denG)

u = U * np.ones_like(t)

t_ol, y_ol, _ = signal.lsim(G_m, U=u, T=t)

plt.figure()
plt.plot(t_ol, y_ol, linewidth=1.8)
plt.grid()
plt.title(f'Resposta a escaló U = {U}')
plt.xlabel('t [s]')
plt.ylabel('Resposta')
plt.show()


# =========================
# 2. BODE (manual)
# =========================

w = np.logspace(-2, 2, 500)
w, H = signal.freqresp((numG, denG), w)

plt.figure()
plt.semilogx(w, 20*np.log10(np.abs(H)))
plt.grid()
plt.title("Bode Magnitud")
plt.xlabel("ω [rad/s]")
plt.ylabel("dB")
plt.show()


# =========================
# 3. ROOT LOCUS (aprox.)
# =========================

K_values = np.linspace(0, 5, 100)
poles_list = []

for K in K_values:
    numK = np.polymul([K], numG)
    den_cl = np.polyadd(denG, numK)
    poles_list.append(np.roots(den_cl))

plt.figure()
for p in poles_list:
    plt.plot(np.real(p), np.imag(p), 'b.', markersize=2)

plt.grid()
plt.title("Root locus (aproximat)")
plt.xlabel("Real")
plt.ylabel("Imag")
plt.show()


# =========================
# 4. PI + LLAÇ TANCAT
# =========================

Kp_ = [10, 10, 10, 10]
Ki_ = [3, 6, 12, 24]

responses = []
labels = []

for Kp, Ki in zip(Kp_, Ki_):

    # PI: (Kp*s + Ki)/s
    numC = [Kp, Ki]
    denC = [1, 0]

    # llaç obert: PI*Gm
    num_ol = np.polymul(numC, numG)
    den_ol = np.polymul(denC, denG)

    # llaç tancat: T = N / (D + N)
    num_cl = num_ol
    den_cl = np.polyadd(den_ol, num_ol)

    sys_cl = signal.lti(num_cl, den_cl)

    t_out, y, _ = signal.lsim(sys_cl, U=U*np.ones_like(t), T=t)

    responses.append(y)
    labels.append(f"Kp={Kp}, Ki={Ki}")


plt.figure()

for y, lab in zip(responses, labels):
    plt.plot(t, y, linewidth=1.8, label=lab)

plt.grid()
plt.title('Resposta PI en llaç tancat')
plt.xlabel('t [s]')
plt.ylabel('Resposta')
plt.legend()
plt.show()