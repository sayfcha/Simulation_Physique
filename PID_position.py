import matplotlib.pyplot as plt
from moteur_univers import MoteurCC

class ControlPID_position:
    def __init__(self, moteur, Kp=1.0, Kd=0.0):
        self.moteur = moteur
        self.Kp = Kp
        self.Kd = Kd
        self.target_position = 0
        self.previous_error = 0

        self.positions = []
        self.tensions = []

    def setTarget(self, position_rad):
        self.target_position = position_rad

    def simulate(self, step):
        # Calcul de l'erreur de position
        error = self.target_position - self.moteur.position
        d_error = (error - self.previous_error) / step

        # Commande PID simplifiée (P + D ici)
        V = self.Kp * error + self.Kd * d_error

        # Appliquer la tension au moteur
        self.moteur.setVoltage(V)
        self.moteur.simulation(step)

        # Mémoriser les données
        self.tensions.append(V)
        self.positions.append(self.moteur.position)
        self.previous_error = error

    def plot(self, temps, label=""):
        # Tracé de la position
        plt.figure()
        plt.plot(temps, self.positions, label=f"Position (rad) {label}")
        plt.xlabel("Temps (s)")
        plt.ylabel("Position θ (rad)")
        plt.title("Réponse du moteur à une consigne de position")
        plt.grid()
        plt.legend()
        plt.show()

        # Tracé de la tension de commande
        plt.figure()
        plt.plot(temps, self.tensions, label=f"Tension de commande {label}", color='orange')
        plt.xlabel("Temps (s)")
        plt.ylabel("Tension (V)")
        plt.title("Tension appliquée par le contrôleur PID")
        plt.grid()
        plt.legend()
        plt.show()


step = 0.01
t_max = 2.0
target = 1.0  # rad

moteur_pd = MoteurCC()
controller = ControlPID_position(moteur_pd, Kp=15, Kd=2)
controller.setTarget(target)

t = 0
temps = []

while t < t_max:
    controller.simulate(step)
    temps.append(t)
    t += step

controller.plot(temps, label=f"Kp=15, Kd=2")
