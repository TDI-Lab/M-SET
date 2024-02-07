import configparser
import math
from pathlib import Path

from scipy.optimize import bisect


class PowerConsumption:

    def __init__(self):
        # fixed param
        self.gravity = 9.8
        self.drag_coefficient = 1.49

        # input fixed param
        self.body_mass = 0
        self.battery_mass = 0
        self.rotor_num = 0
        self.rotor_dia = 0
        self.projected_body = 0
        self.projected_battery = 0
        self.power_efficiency = 0

        # input varying param
        self.air_density = 0
        self.air_speed = 0
        self.ground_speed = 0

        # output results
        self.drag_force = 0
        self.pitch = 0
        self.thrust = 0
        self.induced_velocity = 0
        self.hover_power = 0
        self.flight_power = 0

    def set_params(self):
        parent_path = Path(__file__).parent.resolve()
        config = configparser.ConfigParser()
        config.read(f'{parent_path}/conf/generation.properties')

        self.body_mass = float(config.get('power', 'bodyMass'))
        self.battery_mass = float(config.get('power', 'batteryMass'))
        self.rotor_num = int(config.get('power', 'rotorNum'))
        self.rotor_dia = float(config.get('power', 'rotorDia'))
        self.projected_body = float(config.get('power', 'projectedBody'))
        self.projected_battery = float(config.get('power', 'projectedBattery'))
        self.power_efficiency = float(config.get('power', 'powerEfficiency'))

        self.air_density = float(config.get('power', 'airDensity'))
        self.air_speed = float(config.get('power', 'airSpeed'))
        self.ground_speed = float(config.get('power', 'groundSpeed'))

    def calc_power(self, speed=None):
        if speed is not None:
            self.ground_speed = speed

        self.drag_force = self.air_density * math.pow(self.air_speed, 2) * \
                          (self.projected_body * self.drag_coefficient + self.projected_battery) / 2
        self.pitch = math.degrees(math.atan(self.drag_force / (self.body_mass + self.battery_mass) / self.gravity))
        self.thrust = (self.body_mass + self.battery_mass) * self.gravity + self.drag_force
        self.induced_velocity = self.calc_induced_velocity()

        self.hover_power = math.pow(self.thrust, 1.5) / (self.power_efficiency * math.sqrt(0.5 * math.pi * math.pow(
            self.rotor_dia, 2) * self.rotor_num * self.air_density))
        self.flight_power = (self.ground_speed * math.sin(math.radians(self.pitch)) + self.induced_velocity) * \
                            self.thrust / self.power_efficiency

    def calc_induced_velocity(self):
        constant = math.pi * math.pow(self.rotor_dia, 2) * self.rotor_num * self.air_density
        a = - 4 * math.pow(self.thrust, 2)
        b = math.pow(constant * self.ground_speed, 2)
        c = math.pow(constant, 2) * 2 * self.ground_speed * math.sin(math.radians(self.pitch))
        d = math.pow(constant, 2)
        vector = [a, 0, b, c, d]

        # Define a polynomial function
        def polynomial_function(x):
            return vector[0] + vector[1] * x + vector[2] * x ** 2 + vector[3] * x ** 3 + vector[4] * x ** 4

        root = bisect(polynomial_function, 0, 20, maxiter=100)
        return root


if __name__ == '__main__':
    power_consume = PowerConsumption()
    power_consume.set_params()
    power_consume.calc_power()

    print("drag force is " + str(power_consume.drag_force) + ' N')
    print("pitch is " + str(power_consume.pitch) + ' degrees')
    print("thrust is " + str(power_consume.thrust) + ' N')
    print("induced velocity is " + str(power_consume.induced_velocity) + ' m/s')
    print("The hover power is " + str(power_consume.hover_power) + ' W')
    print("The flight power is " + str(power_consume.flight_power) + ' W')
