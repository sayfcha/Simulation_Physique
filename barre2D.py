import pygame
from pygame.locals import *
from vector3D import Vector3D as V3D
from particule import Particule
from univers import Univers
from math import sin, cos
import matplotlib.pyplot as plt

# Classe modélisant une barre rigide en 2D
class Barre2D(object):
    def __init__(self, mass=1, long=1, theta=0, pos=V3D(), fixed=False, color='red', nom='barre'):
        self.mass = mass
        self.long = long
        self.color = color
        self.nom = nom
        self.fixed = fixed

        # Variables d'état angulaires
        self.theta = theta      # angle initial (en rad)
        self.omega = 0          # vitesse angulaire
        self.alpha = 0          # accélération angulaire

        self.pivot = pos        # position du pivot (point fixe)
        self.pos = self.computeCenter()  # calcul initial du centre de masse

        self.forces = V3D()     # somme des forces (inutile ici mais gardée pour extensibilité)
        self.torque = 0         # couple externe total

        # Historique pour affichage matplotlib
        self.positions = [self.pos]
        self.thetas = [theta]

    def computeCenter(self):
        """Calcule la position du centre de masse à partir du pivot et de l'angle."""
        dx = (self.long / 2) * sin(self.theta)
        dy = -(self.long / 2) * cos(self.theta)
        return V3D(self.pivot.x + dx, self.pivot.y + dy)

    def getExtremities(self):
        """Retourne les extrémités de la barre (pour affichage graphique)."""
        dx = (self.long / 2) * sin(self.theta)
        dy = -(self.long / 2) * cos(self.theta)
        A = V3D(self.pos.x - dx, self.pos.y - dy)
        B = V3D(self.pos.x + dx, self.pos.y + dy)
        return A, B

    def applyEffort(self, Force=V3D(), Torque=V3D(), Point=0):
        """Accumule les efforts appliqués à la barre (ici uniquement le couple externe)."""
        self.forces += Force
        self.torque += Torque.z

    def simulate(self, step):
        if not self.fixed:
            # Moment d'inertie d’une barre par rapport à un pivot à l’extrémité : I = (1/3) mL²
            I = (1/3) * self.mass * self.long**2

            # Bras de levier : du pivot au centre de masse
            bras = self.pos - self.pivot
            poids = V3D(0, -9.81 * self.mass, 0)
            torque_gravite = (bras * poids).z

            total_torque = self.torque + torque_gravite

            # Intégration explicite d’Euler
            self.alpha = total_torque / I
            self.omega += self.alpha * step
            self.theta += self.omega * step

            # Mise à jour du centre de masse
            self.pos = self.computeCenter()

        # Historique
        self.positions.append(self.pos)
        self.thetas.append(self.theta)
        self.forces = V3D()
        self.torque = 0

    def plotRot(self):
        """Trace l’évolution angulaire pour matplotlib."""
        plt.plot(self.thetas, label=f'{self.nom} θ(t)')

    def gameDraw(self, scale, screen):
        """Affichage visuel de la barre dans Pygame."""
        A, B = self.getExtremities()
        X1 = int(scale * A.x)
        Y1 = int(scale * A.y)
        X2 = int(scale * B.x)
        Y2 = int(scale * B.y)
        H = screen.get_height()
        pygame.draw.line(screen, pygame.Color(self.color), (X1, H - Y1), (X2, H - Y2), 4)


# Générateur pour la Gravité appliquée à une Barre2D 
class GravityBarre2D:
    def __init__(self, g=V3D(0, -9.81), name="gravity", active=True):
        self.g = g
        self.name = name
        self.active = active

    def setForce(self, agent):
        if not self.active:
            return
        if hasattr(agent, 'mass') and hasattr(agent, 'applyEffort'):
            poids = self.g * agent.mass
            bras = agent.pos - agent.pivot
            torque = (bras * poids).z
            agent.applyEffort(Force=poids, Torque=V3D(0, 0, torque))


# Générateur : Ressort-amortisseur en rotation entre deux barres 
class TorsionSpringDamper:
    def __init__(self, barre1, barre2, k=10.0, c=1.0, theta0=0.0, active=True, name="TorsionSpringDamper"):
        self.barre1 = barre1
        self.barre2 = barre2
        self.k = k
        self.c = c
        self.theta0 = theta0
        self.active = active
        self.name = name

    def setForce(self, agent):
        if not self.active or agent not in [self.barre1, self.barre2]:
            return

        dtheta = (self.barre2.theta - self.barre1.theta) - self.theta0
        domega = self.barre2.omega - self.barre1.omega
        torque = self.k * dtheta + self.c * domega

        if agent == self.barre1:
            agent.applyEffort(Torque=V3D(0, 0, +torque))
        elif agent == self.barre2:
            agent.applyEffort(Torque=V3D(0, 0, -torque))


# fonction simulation d'un pas de temps 
def simulateAllBarre2D(U):
    for agent in U.population:
        for source in U.generators:
            source.setForce(agent)
        agent.simulate(U.step)
    U.time.append(U.time[-1] + U.step)


#main
if __name__ == '__main__':
    pygame.init()
    U = Univers(name="Pendule", step=0.01, game=True, gameDimensions=(800, 600), dimensions=(20, 20))

    # Barre libre  de longeur 2L (non couplée)
    barre = Barre2D(mass=1, long=4, theta=1.0, pos=V3D(10, 15), color='red', nom='Pendule')
    U.population.append(barre)

    # Pendule simple équivalent (masse ponctuelle suspendue à L = 2)
    pendule_simple = Barre2D(mass=1, long=2, theta=1.0, pos=V3D(6, 15), color='purple', nom='PenduleSimple')
    U.population.append(pendule_simple)


    # Barres couplées
    b1 = Barre2D(mass=1, long=4, theta=1.0, pos=V3D(8, 15), color='blue', nom='Pendule1')
    b2 = Barre2D(mass=1, long=4, theta=0.5, pos=V3D(12, 15), color='green', nom='Pendule2')
    U.population.extend([b1, b2])

    # Ajout des générateurs d'efforts
    U.addGenerators(GravityBarre2D())  # gravité
    U.addGenerators(TorsionSpringDamper(b1, b2, k=5.0, c=0.5))  # couplage entre b1 et b2

    # === Boucle graphique ===
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode(U.gameDimensions)
    running = True
    steps = 0
    max_steps = 600

    while running:
        screen.fill((240, 240, 240))
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        simulateAllBarre2D(U)

        for obj in U.population:
            obj.gameDraw(U.scale, screen)

        # Légende graphique
        font = pygame.font.SysFont("Arial", 18)
        legend_texts = [
            ("Pendule (libre longueur L)", 'red'),
            ("Pendule (libre longueur 2L)", 'purple'),
            ("Pendule1 (couplé)", 'blue'),
            ("Pendule2 (couplé)", 'green'),
        ]
        for i, (label, color) in enumerate(legend_texts):
            text_surface = font.render(label, True, pygame.Color(color))
            screen.blit(text_surface, (10, 10 + i * 20))

        pygame.display.flip()
        clock.tick(U.gameFPS)
        steps += 1
        if steps >= max_steps:
            running = False

    pygame.quit()
     
    # on plot les résultats
    plt.figure("Pendule libre")
    barre.plotRot()
    plt.xlabel("Temps (pas)")
    plt.ylabel("Angle θ (rad)")
    plt.title("Évolution angulaire du pendule libre")
    plt.grid()
    plt.legend()

    plt.figure("Pendules couplés")
    b1.plotRot()
    b2.plotRot()
    plt.xlabel("Temps (pas)")
    plt.ylabel("Angle θ (rad)")
    plt.title("Évolution angulaire des pendules couplés")
    plt.grid()
    plt.legend()
    plt.show()

