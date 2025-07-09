import matplotlib.pyplot as plt
from math import sin, cos
from vector3D import Vector3D as V3D
from univers import Univers
from particule import Particule
from moteur_univers import MoteurCC
from univers import SpringDamper


# PARAMÈTRES DE SIMULATION


tensions = [0.2, 0.4, 0.6, 0.8, 1.0]  # Tensions appliquées au moteur
step = 0.01                          # Pas de temps pour l'intégration
duree = 5.0                          # Durée de simulation pour chaque tension (doit permettre d’atteindre un régime permanent)

# Listes pour stocker les résultats
distances = []    # Distance finale simulée (élongation)
vitesses = []     # Vitesse angulaire finale atteinte
theoriques = []   # Élongation calculée théoriquement

# Paramètres physiques
rayon_initial = 1.0   # Longueur initiale du ressort
k = 1.0               # Raideur du ressort
c = 0.5               # Coefficient d'amortissement
m = 1.0               # Masse de la particule


# On fait la simulation pour chaque tension


for tension in tensions:
    # Création d'un nouvel univers pour chaque tension
    U = Univers(name=f"centrifugeuse_U={tension}", step=step)

    # Particule fixe représentant le centre de rotation
    centre = Particule(p0=V3D(0, 0, 0), fix=True)

    # Moteur CC simulé en boucle ouverte
    moteur = MoteurCC()
    moteur.setVoltage(tension)

    #  moteur pour pouvoir l'ajouter à l'univers
    class MoteurSimulable:
        def __init__(self, moteur):
            self.moteur = moteur
            self.omega = []

        def simulate(self, step):
            self.moteur.simulation(step)
            self.omega.append(self.moteur.speed)

    moteur_simul = MoteurSimulable(moteur)

    # Initialisation de la particule à distance rayon_initial avec vitesse tangentielle
    omega_init = tension / moteur.R  # Approximation : Ω = U / R
    position_init = V3D(rayon_initial, 0, 0)  # Position initiale sur l’axe X
    vitesse_init = V3D(0, rayon_initial * omega_init, 0)  # Vitesse tangentielle (sur Y) : v = r * Ω

    particule = Particule(p0=position_init, v0=vitesse_init, mass=m)

    # Liaison ressort + amortisseur entre le centre et la particule
    lien = SpringDamper(centre, particule, k=k, c=c, l0=rayon_initial)

    # Ajout des objets dans l’univers
    U.addParticule(centre, particule, moteur_simul)
    U.addGenerators(lien)

    # Simulation du système pendant la durée spécifiée
    U.simulateFor(duree)

    
    # mesures finales à la fin de simulation

    final_d = particule.getPosition().mod()               # Distance au centre (norme du vecteur position)
    vitesse_tangentielle = particule.getSpeed().mod()     # Norme de la vitesse
    final_omega = vitesse_tangentielle / final_d          # Ω estimée : v = r * Ω => Ω = v / r

    distances.append(final_d)         # Sauvegarde de la distance mesurée
    vitesses.append(final_omega)      # Sauvegarde de la vitesse angulaire

    # Calcul théorique à l’équilibre : r = (k * l0) / (k - m * ω²)
    try:
        theorique = (k * rayon_initial) / (k - m * final_omega**2)
    except ZeroDivisionError:
        theorique = float('inf')  # Cas extrême où ω² = k/m

    theoriques.append(theorique)  # Sauvegarde

# on plot les resultats

plt.figure()
plt.plot(vitesses, distances, 'o-', label="Simulation d(Ω)")         # Résultats simulés
plt.plot(vitesses, theoriques, 's--', label="Théorie d(Ω)")          # Résultats théoriques
plt.xlabel("Vitesse angulaire Ω (rad/s)")
plt.ylabel("Distance d (m)")
plt.title("Distance radiale vs vitesse de rotation : Simulation vs Théorie")
plt.grid()
plt.legend()
plt.show()
