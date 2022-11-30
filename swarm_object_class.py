import numpy as np

from agent_class import Agent
from typing import List, Union


class SwarmObject:
    def __init__(self):
        # ---- Parameters ---- #
        # Distance tolerance around a waypoint for it to be considered as reached
        self.distance_to_waypoint_threshold: float = 0.05   # (m)

        # Obstacle detection distance
        self.xy_auto_avoid_d0: float = 0.85     # (m)

        # ---- Attributes initialization ---- #
        self.manual_x: float = 0.0
        self.manual_y: float = 0.0
        self.manual_z: float = 0.0
        self.manual_yaw: float = 0.0        # (°)
        self.manual_flight_agents_list: List[str] = []

        self.pursuit_eta: float = 0.0       # (rad)

        self.ready_to_fly: bool = False
        self.swarm_leader: Union[None, str] = None
        self.target: Union[None, str] = None
        self.chaser: Union[None, str] = None
        self.swarm_agent_list: List[Agent] = []
        self.agent_name_list: List[str] = []
        self.waypoint_number: Union[None, int] = None

    def add_agent(self, agent: Agent):
        self.swarm_agent_list.append(agent)
        self.agent_name_list.append(agent.name)

    def remove_agent(self, agent: Agent):
        if self.swarm_leader == agent.name:
            self.swarm_leader = None
        try:
            name_index = self.agent_name_list.index(agent.name)
            _ = self.agent_name_list.pop(name_index)
            __ = self.swarm_agent_list.pop(name_index)
        except ValueError:
            print('-- Warning -- An exception occurred during agent removal attempt')

    def assign_swarm_leader(self, cf_name: str):
        self.swarm_leader = cf_name

    def assign_target(self, cf_name: str):
        self.target = cf_name

    def assign_chaser(self, cf_name: str):
        self.chaser = cf_name

    def flight_sequence(self):
        if self.ready_to_fly:
            for agent in self.swarm_agent_list:
                if agent.enabled and agent.state != 'Not flying':
                    if agent.state == 'Standby':
                        self.standby_control_law(agent)

                    if agent.state == 'Takeoff':
                        self.takeoff_control_law(agent)

                    if agent.state == 'Land':
                        self.landing_control_law(agent)

                    if agent.state == 'Manual':
                        self.manual_control_law(agent)

                    if agent.state == 'xy_consensus':
                        self.xy_consensus_control_law(agent)

                    if agent.state == 'z_consensus':
                        self.z_consensus_control_law(agent)

                    if agent.state == 'Wingman':
                        self.wingman_control_law(agent)

                    if agent.state == 'Back_to_init':
                        self.back_to_initial_position_ctl_law(agent)

                    if agent.state == 'Pursuit':
                        self.pursuit_control_law(agent)
                else:
                    agent.cf.commander.send_stop_setpoint()

        elif all([(agent.battery_test_passed and agent.extpos_test_passed) for agent in self.swarm_agent_list]):
            self.ready_to_fly = True
            print('Crazyflie connection recap :')
            for agent in self.swarm_agent_list:
                agent.setup_finished = True
                print('    -', agent.name, ':')
                if agent.extpos.x >= 0:
                    print('        x = ', round(agent.extpos.x, 3), 'm')
                else:
                    print('        x =', round(agent.extpos.x, 3), 'm')
                if agent.extpos.y >= 0:
                    print('        y = ', round(agent.extpos.y, 3), 'm')
                else:
                    print('        y =', round(agent.extpos.y, 3), 'm')
                if agent.extpos.z >= 0:
                    print('        z = ', round(agent.extpos.z, 3), 'm')
                else:
                    print('        z =', round(agent.extpos.z, 3), 'm')
                print('        Battery level =', round(agent.initial_battery_level), '%')
                if agent.enabled:
                    print('        Flight enabled')
                else:
                    print('        -- Warning -- : Flight disabled')

    def manual_control_law(self, agent: Agent):
        kp = 0.5
        ks = 0.20
        if self.manual_x >= 0:
            agent.standby_position[0] = agent.extpos.x - kp * np.sqrt(self.manual_x)
            if agent.standby_position[0] < agent.x_boundaries[0] + ks:
                agent.standby_position[0] = agent.x_boundaries[0] + ks
        else:
            agent.standby_position[0] = agent.extpos.x + kp * np.sqrt(-self.manual_x)
            if agent.standby_position[0] > agent.x_boundaries[1] - ks:
                agent.standby_position[0] = agent.x_boundaries[1] - ks
        if self.manual_y >= 0:
            agent.standby_position[1] = agent.extpos.y - kp * np.sqrt(self.manual_y)
            if agent.standby_position[1] < agent.y_boundaries[0] + ks:
                agent.standby_position[1] = agent.y_boundaries[0] + ks
        else:
            agent.standby_position[1] = agent.extpos.y + kp * np.sqrt(-self.manual_y)
            if agent.standby_position[1] > agent.y_boundaries[1] - ks:
                agent.standby_position[1] = agent.y_boundaries[1] - ks
        agent.standby_position[2] = self.manual_z * (0.90 * agent.z_boundaries[1])
        if agent.standby_position[2] > agent.z_boundaries[1] - ks:
            agent.standby_position[2] = agent.z_boundaries[1] - ks
        agent.standby_yaw = self.manual_yaw
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = distance([agent.extpos.x, agent.extpos.y,
                      agent.extpos.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Takeoff completed')
            agent.is_flying = True
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = vertical_distance(agent.extpos.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Landing completed')
            agent.stop()
            self.remove_agent(agent)

    def xy_consensus_control_law(self, agent: Agent):
        if not self.swarm_leader:
            in_flight_agents = [agt for agt in self.swarm_agent_list if agt.state != 'Not flying']
            roll, pitch = get_xy_consensus_attitude(agent, in_flight_agents)
            thrust = thrust_control_law(agent, agent.takeoff_height)
            yaw_rate = yaw_rate_control_law(agent, 0)
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                       agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                       'None', 'None', 'None',
                                       roll, pitch, yaw_rate, thrust])
            agent.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)
        else:
            print('-- Warning -- ', agent.name, ': no leader set for xy consensus')
            agent.standby()

    def z_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt
                            for agt in self.swarm_agent_list if agt.state != 'Not flying' and agt != agent]
        kp = 1
        vx = kp * (agent.z_consensus_xy_position[0] - agent.extpos.x)
        vy = kp * (agent.z_consensus_xy_position[1] - agent.extpos.y)
        vz = kp * (sum([agt.extpos.z - agent.extpos.z
                        for agt in in_flight_agents if agt.name in agent.consensus_connectivity]))
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def standby_control_law(self, agent: Agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.standby_position[0], agent.standby_position[1]]
        omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                 [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.standby_position[2] - agent.extpos.z
        yaw_rate = yaw_rate_control_law(agent, 0)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def wingman_control_law(self, agent):
        if not self.swarm_leader:
            print(' -- Warning -- ', agent.name, ': no swarm leader to follow, standby mode triggered')
            agent.standby()
        else:
            agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
            leader_agent = [agt for agt in self.swarm_agent_list if agt.name == self.swarm_leader]
            objective = [leader_agent[0].extpos.x, leader_agent[0].extpos.y]
            omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                     [agent.x_boundaries[1], agent.y_boundaries[1]]))
            vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                     objective, omega, self.xy_auto_avoid_d0)
            vz = agent.wingman_z - agent.extpos.z
            yaw_rate = yaw_rate_control_law(agent, leader_agent[0].yaw)
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                       agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                       vx, vy, vz,
                                       'None', 'None', 'None', 'None'])
            agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def back_to_initial_position_ctl_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.initial_position[0], agent.initial_position[1]]
        omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                 [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.takeoff_height - agent.extpos.z
        yaw_rate = yaw_rate_control_law(agent, agent.back_to_init_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def pursuit_control_law(self, agent: Agent):
        if agent.name == self.target:
            kp = 0.30
            ks = 0.20
            if self.manual_x >= 0:
                agent.standby_position[0] = agent.extpos.x - kp * np.sqrt(self.manual_x)
                if agent.standby_position[0] < agent.x_boundaries[0] + ks:
                    agent.standby_position[0] = agent.x_boundaries[0] + ks
            else:
                agent.standby_position[0] = agent.extpos.x + kp * np.sqrt(-self.manual_x)
                if agent.standby_position[0] > agent.x_boundaries[1] - ks:
                    agent.standby_position[0] = agent.x_boundaries[1] - ks
            if self.manual_y >= 0:
                agent.standby_position[1] = agent.extpos.y - kp * np.sqrt(self.manual_y)
                if agent.standby_position[1] < agent.y_boundaries[0] + ks:
                    agent.standby_position[1] = agent.y_boundaries[0] + ks
            else:
                agent.standby_position[1] = agent.extpos.y + kp * np.sqrt(-self.manual_y)
                if agent.standby_position[1] > agent.y_boundaries[1] - ks:
                    agent.standby_position[1] = agent.y_boundaries[1] - ks
            agent.standby_position[2] = 0.25
            agent.standby_yaw = 0
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                       agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                       'None', 'None', 'None',
                                       'None', 'None', 'None', 'None'])
            agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                      agent.standby_position[2], agent.standby_yaw)
        elif agent.name == self.chaser:
            target = [agt for agt in self.swarm_agent_list if agt.name == self.target]
            d = horizontal_distance([agent.extpos.x, agent.extpos.y], [target[0].extpos.x, target[0].extpos.y])  # (m)
            if d <= 0.1:
                print('Impact')
                target[0].land()
                agent.standby()
            else:
                # -- Roll control law -- #
                vy = 0          # (m/s)
                kp = 1
                measured_vy = (- agent.velocity[0] * np.sin(agent.yaw * np.pi / 180)
                               + agent.velocity[1] * np.cos(agent.yaw * np.pi / 180))   # (m/s)
                roll = - round(kp * (vy - measured_vy) * 180 / np.pi)                   # (°)
                max_roll = 30  # (°)
                if roll > max_roll:
                    roll = max_roll
                elif roll < - max_roll:
                    roll = - max_roll

                # -- Pitch control law -- #
                vx = 1       # (m/s)
                kp = 1
                measured_vx = (agent.velocity[0] * np.cos(agent.yaw * np.pi / 180)
                               + agent.velocity[1] * np.sin(agent.yaw * np.pi / 180))   # (m/s)
                pitch = round(kp * (vx - measured_vx) * 180 / np.pi)                    # (°)
                max_pitch = 30  # (°)
                if pitch > max_pitch:
                    pitch = max_pitch
                elif pitch < - max_pitch:
                    pitch = - max_pitch

                # -- Yaw rate control law -- #
                closing_speed_x = target[0].velocity[0] - agent.velocity[0]
                closing_speed_y = target[0].velocity[1] - agent.velocity[1]
                closing_speed = np.sqrt(closing_speed_x ** 2 + closing_speed_y ** 2)

                a = 27
                eta = np.arctan2(target[0].extpos.y - agent.extpos.y,
                                 target[0].extpos.x - agent.extpos.x)               # (rad)
                yaw = (agent.yaw * np.pi / 180) + (a * closing_speed * (eta - self.pursuit_eta))    # (rad)
                self.pursuit_eta = eta                                              # (rad)
                yaw_rate = yaw_rate_control_law(agent, yaw * 180 / np.pi)           # (°/s)

                # -- Thrust control law -- #
                z = 0.5         # (m)
                thrust = thrust_control_law(agent, z)   # (PWM)

                # -- Volume borders security check -- #
                #       Interrupts the chase to keep the UAV within the flight volume boundaries
                ks = 0.30
                if (agent.extpos.x < agent.x_boundaries[0] + ks
                        or agent.extpos.x > agent.x_boundaries[1] - ks
                        or agent.extpos.y < agent.y_boundaries[0] + ks
                        or agent.extpos.y > agent.y_boundaries[1] - ks):
                    x = 6.5 * agent.extpos.x / 8        # (m)
                    y = 6.5 * agent.extpos.y / 8        # (m)
                    yaw = (180 / np.pi) * (np.pi + np.arctan2(agent.extpos.y, agent.extpos.x))    # (°)
                    agent.cf.commander.send_position_setpoint(x, y, z, yaw)
                    agent.csv_logger.writerow([agent.name, agent.timestamp,
                                               agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                               agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                               'None', 'None', 'None',
                                               'None', 'None', 'None', 'None'])
                else:
                    agent.csv_logger.writerow([agent.name, agent.timestamp,
                                               agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                               agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                               'None', 'None', 'None',
                                               roll, pitch, yaw_rate, thrust])
                    agent.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)
        else:
            agent.land()


def get_auto_avoid_velocity_command(agent: Agent, x_limits: [float] * 2, y_limits: [float] * 2,
                                    agents_to_avoid: List[Agent], objective: [float] * 2, omega: float, d0: float):
    vx = 0.0
    vy = 0.0

    # Agents to avoid
    kpo = 2.5
    kvo = 1
    for agt in agents_to_avoid:
        v = np.sqrt(agt.velocity[0] ** 2 + agt.velocity[1] ** 2)
        if v > 0.25:
            kv2 = (kvo * v) ** 2
            xb = agt.extpos.x + kvo * agt.velocity[0]
            yb = agt.extpos.y + kvo * agt.velocity[1]
            xc = agt.extpos.x + (((xb - agt.extpos.x) / (kv2 + 0.001))
                                 * ((xb - agt.extpos.x) * (agent.extpos.x - agt.extpos.x)
                                    + (yb - agt.extpos.y) * (agent.extpos.y - agt.extpos.y)))
            yc = agt.extpos.y + ((xb - agt.extpos.x) * (agent.extpos.x - agt.extpos.x)
                                 + (yb - agt.extpos.y) * (agent.extpos.y - agt.extpos.y)) / (kv2 + 0.001)

            if ((agt.extpos.x < xc < xb or xb < xc < agt.extpos.x)
                    and (agt.extpos.y < yc < yb or yb < yc < agt.extpos.y)):
                d = horizontal_distance([agent.extpos.x, agent.extpos.y], [xc, yc])
                if d <= d0:
                    vx = vx - (((xc - agent.extpos.x) / (d + 0.001))
                               * kpo * (np.exp(-d) - np.exp(-d0)))
                    vy = vy - (((yc - agent.extpos.y) / (d + 0.001))
                               * kpo * (np.exp(-d) - np.exp(-d0)))
            else:
                da = horizontal_distance([agent.extpos.x, agent.extpos.y], [agt.extpos.x, agt.extpos.y])
                db = horizontal_distance([agent.extpos.x, agent.extpos.y], [xb, yb])
                if da < db and da <= d0:
                    vx = vx - (((agt.extpos.x - agent.extpos.x) / (da + 0.001))
                               * kpo * (np.exp(-da) - np.exp(-d0)))
                    vy = vy - (((agt.extpos.y - agent.extpos.y) / (da + 0.001))
                               * kpo * (np.exp(-da) - np.exp(-d0)))
                elif db < da and db < d0:
                    vx = vx - (((xb - agent.extpos.x) / (db + 0.001))
                               * kpo * (np.exp(-db) - np.exp(-d0)))
                    vy = vy - (((yb - agent.extpos.y) / (db + 0.001))
                               * kpo * (np.exp(-db) - np.exp(-d0)))
        else:
            d = horizontal_distance([agent.extpos.x, agent.extpos.y], [agt.extpos.x, agt.extpos.y])
            if d <= d0:
                vx = vx - (((agt.extpos.x - agent.extpos.x) / (d + 0.001))
                           * kpo * (np.exp(-d) - np.exp(-d0)))
                vy = vy - (((agt.extpos.y - agent.extpos.y) / (d + 0.001))
                           * kpo * (np.exp(-d) - np.exp(-d0)))

    # Objective
    kpg = 1
    distance_to_objective = horizontal_distance([agent.extpos.x, agent.extpos.y],
                                                [objective[0],
                                                 objective[1]])
    d1 = np.pi / (2 * omega)
    vx = vx + ((objective[0] - agent.extpos.x) * kpg
               / (2 * d1 * np.sqrt((distance_to_objective + 0.001) / d1)))
    vy = vy + ((objective[1] - agent.extpos.y) * kpg
               / (2 * d1 * np.sqrt((distance_to_objective + 0.001) / d1)))

    # Borders
    x_min = x_limits[0] + 0.2 * (x_limits[1] - x_limits[0])
    y_min = y_limits[0] + 0.2 * (y_limits[1] - y_limits[0])
    x_max = x_limits[1] - 0.2 * (x_limits[1] - x_limits[0])
    y_max = y_limits[1] - 0.2 * (y_limits[1] - y_limits[0])
    if agent.extpos.x < x_min and vx < 0:
        vx = 0
    if agent.extpos.y < y_min and vy < 0:
        vy = 0
    if agent.extpos.x > x_max and vx > 0:
        vx = 0
    if agent.extpos.y > y_max and vy > 0:
        vy = 0

    return vx, vy


def get_xy_consensus_attitude(agent: Agent, agents_list: List[Agent]):
    measured_yaw = agent.yaw * np.pi / 180  # Convert from degrees to radians

    dx = agent.xy_consensus_offset_from_leader[0]
    dy = agent.xy_consensus_offset_from_leader[1]

    connected_agents = [agt for agt in agents_list
                        if agt.name in agent.consensus_connectivity and agt != agent]

    kp = 0.25
    eta = 1
    x_dot_dot = - sum([kp * (agent.extpos.x - agt.extpos.x + (agt.xy_consensus_offset_from_leader[0] - dx))
                       + eta * kp * (agent.velocity[0] - agt.velocity[0]) for agt in connected_agents])
    y_dot_dot = sum([kp * (agent.extpos.y - agt.extpos.y + (agt.xy_consensus_offset_from_leader[1] - dy))
                     + eta * kp * (agent.velocity[1] - agt.velocity[1]) for agt in connected_agents])
    pitch = (x_dot_dot * np.cos(measured_yaw) + y_dot_dot * np.sin(measured_yaw)) * 180 / np.pi
    roll = (- x_dot_dot * np.sin(measured_yaw) + y_dot_dot * np.cos(measured_yaw)) * 180 / np.pi

    max_roll = 20  # (°)
    max_pitch = 20  # (°)

    if roll > max_roll:
        roll = max_roll
    elif roll < -max_roll:
        roll = -max_roll

    if pitch > max_pitch:
        pitch = max_pitch
    elif pitch < -max_pitch:
        pitch = -max_pitch

    return roll, pitch


def thrust_control_law(agent: Agent, targeted_z: float) -> int:
    """
    Controls the height of a drone by adjusting its thrust via a PID
    """

    z_kp = 32500
    z_ki = 8125
    z_kd = 16250
    thrust_at_steady_state = 38000

    z_error = targeted_z - agent.extpos.z
    pz = z_kp * z_error
    iz = agent.previous_iz + z_ki * z_error * agent.delta_t
    dz = - z_kd * agent.velocity[2]
    thrust = thrust_at_steady_state + pz + iz + dz

    thrust = round(thrust)
    if thrust > 65000:
        thrust = 65000
    elif thrust < 0:
        thrust = 0

    agent.previous_iz = iz
    return thrust


def yaw_rate_control_law(agent: Agent, targeted_yaw: float) -> float:
    """
    Returns a yaw_rate command (°/s) for a UAV to match its targeted yaw (°)
    """

    yaw_kp = 5
    targeted_yaw = targeted_yaw * np.pi / 180  # (° -> rad)
    targeted_yaw = targeted_yaw % (2 * np.pi)
    if targeted_yaw > np.pi:
        targeted_yaw = targeted_yaw - (2 * np.pi)

    measured_yaw = agent.yaw * np.pi / 180
    measured_yaw = measured_yaw % (2 * np.pi)
    if measured_yaw > np.pi:
        measured_yaw = measured_yaw - (2 * np.pi)

    yaw_error_0 = targeted_yaw - measured_yaw
    yaw_error_1 = targeted_yaw - measured_yaw + 2 * np.pi
    yaw_error_2 = targeted_yaw - measured_yaw - 2 * np.pi

    if abs(yaw_error_1) < abs(yaw_error_0) and abs(yaw_error_1) < abs(yaw_error_2):
        yaw_error = yaw_error_1
    elif abs(yaw_error_2) < abs(yaw_error_0) and abs(yaw_error_2) < abs(yaw_error_1):
        yaw_error = yaw_error_2
    else:
        yaw_error = yaw_error_0

    yaw_rate = - round(yaw_kp * yaw_error * 180 / np.pi)  # (°/s)

    max_yaw_rate = 180  # (°/s)
    if yaw_rate > max_yaw_rate:
        yaw_rate = max_yaw_rate
    elif yaw_rate < -max_yaw_rate:
        yaw_rate = -max_yaw_rate
    return yaw_rate


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = np.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = np.sqrt((position_1_z - position_2_z) ** 2)
    return vd


def horizontal_distance(position_1_xy: List[float], position_2_xy: List[float]):
    hd = np.sqrt((position_1_xy[0] - position_2_xy[0]) ** 2
                 + (position_1_xy[1] - position_2_xy[1]) ** 2)
    return hd
