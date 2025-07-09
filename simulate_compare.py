import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from turtlebots2 import TurtleBot

# Deux robots, un par mode de commande
bot_kin = TurtleBot(P0=V3D(0, 0), mode='kinematic')
bot_mot = TurtleBot(P0=V3D(0, 2), mode='motor')

# Commande constante : tourner sur une courbe
bot_kin.set_wheel_speeds(5, 2)
bot_mot.set_motor_voltages(5, 2)

# Simulation
dt = 0.01
T = 10
steps = int(T / dt)
traj_kin = []
traj_mot = []

for _ in range(steps):
    bot_kin.move(dt)
    bot_mot.move(dt)
    traj_kin.append(bot_kin.position)
    traj_mot.append(bot_mot.position)

# Tracé
plt.figure()
plt.plot([p.x for p in traj_kin], [p.y for p in traj_kin], label="Ciné. roue")
plt.plot([p.x for p in traj_mot], [p.y for p in traj_mot], label="Moteur CC")
plt.title("Comparaison de suivi de trajectoire")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid()
plt.axis('equal')
plt.show()
