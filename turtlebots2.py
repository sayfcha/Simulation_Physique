from vector3D import Vector3D as V3D
from moteurcc import MoteurCC
from math import pi

class TurtleBot:
    def __init__(self, P0=V3D(), R0=0, name='bot', color='blue',
                 r=0.05, L=0.15, mode='kinematic', vmax=1.0, wmax=pi/10):

        self.position = P0
        self.orientation = R0
        self.pose = [(P0, R0)]
        self.name = name
        self.color = color

        self.wheel_radius = r
        self.wheel_dist = L
        self.mode = mode  # 'kinematic' ou 'motor'

        self.vmax = vmax
        self.wmax = wmax

        self.wL = 0
        self.wR = 0

        if self.mode == 'motor':
            self.motorL = MoteurCC()
            self.motorR = MoteurCC()

        self.v = 0
        self.omega = 0

    def move(self, step=0.01):
        if self.mode == 'motor':
            self.motorL.simulation(step)
            self.motorR.simulation(step)
            self.wL = self.motorL.speed
            self.wR = self.motorR.speed
        # Sinon : les vitesses wL/wR sont déjà fixées

        # Conversion cinématique
        self.v = (self.wheel_radius / 2) * (self.wL + self.wR)
        self.omega = (self.wheel_radius / self.wheel_dist) * (self.wR - self.wL)

        # Mise à jour de l'état
        self.orientation += self.omega * step
        dx = self.v * step
        dpos = V3D(dx, 0, 0).rotZ(self.orientation)
        self.position += dpos
        self.pose.append((self.position, self.orientation))

    def set_wheel_speeds(self, wL, wR):
        self.wL = wL
        self.wR = wR

    def set_motor_voltages(self, VL, VR):
        if self.mode == 'motor':
            self.motorL.setVoltage(VL)
            self.motorR.setVoltage(VR)
