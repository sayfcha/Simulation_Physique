import matplotlib.pyplot as plt
import numpy as np

# CLASSE MOTEUR CC
class MoteurCC:
    def __init__(self, R=1, L=0.001, kc=0.01, ke=0.01, J=0.01, f=0.1):
        self.R = R
        self.L = L
        self.kc = kc
        self.ke = ke
        self.J = J
        self.f = f

        self.voltage = 0
        self.current = 0
        self.speed = 0
        self.position = 0
        self.torque = 0

        self.intensites = []
        self.vitesses = []
        self.positions = []

    def setVoltage(self, V):  #applique une tension Um(t) au moteur
        self.voltage = V

    def simulation(self, step):  # intègre l’évolution de la vitesse et du courant 
        self.current = (self.voltage - self.ke * self.speed) / self.R
        self.torque = self.kc * self.current
        dOmega = (self.torque - self.f * self.speed) / self.J
        self.speed += dOmega * step
        self.position += self.speed * step

        self.intensites.append(self.current)
        self.vitesses.append(self.speed)
        self.positions.append(self.position)

# Simulation en boucle ouvert
m = MoteurCC()
step = 0.01
t = 0
temps = []
U0 = 0.1  # même échelon que la consigne utilisée après

while t < 2:
    m.setVoltage(U0)
    m.simulation(step)
    temps.append(t)
    t += step

# Solution analytique
R = m.R
kc = m.kc
ke = m.ke
J = m.J
f = m.f

K = (kc / R) * U0 / (f + (kc * ke / R))
tau = J / (f + (kc * ke / R))
omega_theorique = [K * (1 - np.exp(-tt / tau)) for tt in temps]

plt.figure()
plt.plot(temps, m.vitesses, label="Simulation numérique", color="blue")
plt.plot(temps, omega_theorique, label="Réponse théorique", linestyle="--", color="orange")
plt.title("Comparaison Simulation vs Théorie (Moteur CC)")
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse Ω (rad/s)")
plt.grid()
plt.legend()
plt.show()

#classe controleur pid 
class ControlPID_vitesse:
    def __init__(self, moteur, Kp=1.0, Ki=0.0):
        self.moteur = moteur
        self.Kp = Kp
        self.Ki = Ki

        self.erreur_cumulee = 0
        self.target_speed = 0

        self.tensions = []
        self.vitesses = []

    def setTarget(self, vitesse):
        self.target_speed = vitesse

    def simulation(self, step):
        erreur = self.target_speed - self.moteur.speed
        self.erreur_cumulee += erreur * step

        V = self.Kp * erreur + self.Ki * self.erreur_cumulee
        self.moteur.setVoltage(V)
        self.moteur.simulation(step)

        self.tensions.append(V)
        self.vitesses.append(self.moteur.speed)

    def plot(self, temps, titre):
        plt.figure()
        plt.plot(temps, self.vitesses, label="Vitesse (rad/s)", color='green')
        plt.title(titre)
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse (rad/s)")
        plt.grid()
        plt.legend()
        plt.show()

#controleur P et PI 
m_bf_P = MoteurCC()
m_bf_PI = MoteurCC()
control_P = ControlPID_vitesse(m_bf_P, Kp=5, Ki=0)  # kp=5 bon compromis entre temps de réponse et stabilité
control_PI = ControlPID_vitesse(m_bf_PI, Kp=5, Ki=20) #ki=20 élimine efficacement l’erreur statique sans trop d’oscillations

t = 0
temps_control = []
target_speed = 0.1
control_P.setTarget(target_speed)
control_PI.setTarget(target_speed)

while t < 2:
    control_P.simulation(step)
    control_PI.simulation(step)
    temps_control.append(t)
    t += step

# === PLOTS ===
control_P.plot(temps_control, "Réponse du moteur avec contrôleur P")
control_PI.plot(temps_control, "Réponse du moteur avec contrôleur PI")

plt.figure()
plt.plot(temps_control, control_PI.tensions, label="Tension PI")
plt.xlabel("Temps (s)")
plt.ylabel("Tension (V)")
plt.title("Tension de commande du contrôleur PI")
plt.grid()
plt.legend()
plt.show()
