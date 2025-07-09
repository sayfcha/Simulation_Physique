from random import random
from vector3D import Vector3D as V3D
from univers import Univers  #  classe Univers
from particule import Particule  
import matplotlib.pyplot as plt

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

    def setVoltage(self, V):
        self.voltage = V

    def simulation(self, step):
        self.current = (self.voltage - self.ke * self.speed) / self.R
        self.torque = self.kc * self.current
        dOmega = (self.torque - self.f * self.speed) / self.J
        self.speed += dOmega * step
        self.position += self.speed * step

        self.intensites.append(self.current)
        self.vitesses.append(self.speed)
        self.positions.append(self.position)

# Contrôleur PID pour moteur CC 
class ControlPID_vitesse:
    def __init__(self, moteur, Kp=1.0, Ki=0.0):
        self.moteur = moteur
        self.Kp = Kp
        self.Ki = Ki
        self.erreur_cumulee = 0
        self.target_speed = 0

    def setTarget(self, vitesse):
        self.target_speed = vitesse

    def simulation(self, step):
        erreur = self.target_speed - self.moteur.speed
        self.erreur_cumulee += erreur * step
        V = self.Kp * erreur + self.Ki * self.erreur_cumulee
        self.moteur.setVoltage(V)
        self.moteur.simulation(step)


class MoteurBO(MoteurCC):
    def __init__(self, voltage=1.0, name="BO", **kwargs):
        super().__init__(**kwargs)
        self.name = name
        self.voltage = voltage
        self.vitesses = []

    def simulate(self, step):
        self.setVoltage(self.voltage)
        self.simulation(step)
        self.vitesses.append(self.speed)

class MoteurBF(ControlPID_vitesse):
    def __init__(self, moteur, Kp=1.0, Ki=0.0, name="BF"):
        super().__init__(moteur, Kp, Ki)
        self.name = name
        self.vitesses = []

    def simulate(self, step):
        super().simulation(step)
        self.vitesses.append(self.moteur.speed)

#on fais la simulation dans l'unievrs 
if __name__ == "__main__":
    # Création de l'univers
    monUnivers = Univers(name="Univers Moteur CC", step=0.01)

    # Création moteur BO (boucle ouverte)
    moteur_bo = MoteurBO(voltage=1.0, name="Boucle Ouverte")

    # Création moteur BF (boucle fermée avec PID)
    moteur_base = MoteurCC()
    moteur_bf = MoteurBF(moteur_base, Kp=5, Ki=20, name="Boucle Fermée")
    moteur_bf.setTarget(1.0)  # cible : 1 rad/s

    # Ajout dans Univers
    monUnivers.addParticule(moteur_bo, moteur_bf)

    # Simulation pendant 2 secondes
    monUnivers.simulateFor(2.0)

    # Création du vecteur temps
    # Temps pour chaque moteur (en fonction du nombre de points enregistrés)
    temps_bo = [i * monUnivers.step for i in range(len(moteur_bo.vitesses))]
    temps_bf = [i * monUnivers.step for i in range(len(moteur_bf.vitesses))]

    # Affichage des vitesses
    plt.figure()
    plt.plot(temps_bo, moteur_bo.vitesses, label="Moteur BO")
    plt.plot(temps_bf, moteur_bf.vitesses, label="Moteur BF (PID)")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (rad/s)")
    plt.title("Réponse moteur CC – Boucle Ouverte vs Fermée dans Univers")
    plt.grid()
    plt.legend()
    plt.show()
