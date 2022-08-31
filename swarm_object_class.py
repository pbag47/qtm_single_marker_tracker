import math
import numpy as np

from agent_class import Agent
from air_base_class import SquareAirBase
from typing import List, Union


class SwarmObject:
    def __init__(self):
        # ---- Parameters ---- #
        # Distance tolerance around a waypoint for it to be considered as reached (m)
        self.distance_to_waypoint_threshold: float = 0.05

        # Obstacle detection distance (m)
        self.xy_auto_avoid_d0: float = 0.50  # 0.75

        # ---- Attributes initialization ---- #
        self.manual_x: float = 0.0
        self.manual_y: float = 0.0
        self.manual_z: float = 0.0
        self.manual_yaw: float = 0.0
        self.manual_flight_agents_list: List[str] = []

        self.ready_to_fly: bool = False
        self.swarm_leader: Union[None, Agent] = None
        self.swarm_agent_list: List[Agent] = []
        self.agent_name_list: List[str] = []
        self.waypoint_number: Union[None, int] = None

        self.air_base_list: List[SquareAirBase] = []

        self.flightmode = 'position'  # Choose between 'position', 'velocity', and 'attitude' flight mode

    def add_agent(self, agent: Agent):
        self.swarm_agent_list.append(agent)
        self.agent_name_list.append(agent.name)

    def assign_swarm_leader(self, cf_name: str):
        self.swarm_leader = cf_name

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

                    if agent.state == 'z_consensus':
                        self.z_consensus_control_law(agent)

                    if agent.state == 'Wingman':
                        self.wingman_control_law(agent)

                    if agent.state == 'Back_to_init':
                        self.back_to_initial_position_ctl_law(agent)

                    if agent.state == 'Land_on_air_base':
                        self.land_on_air_base(agent)

                    if agent.state == 'Sleep_on_air_base':
                        self.sleep_on_air_base(agent)

                    if agent.state == 'Takeoff_from_air_base':
                        self.takeoff_from_air_base(agent)
                else:
                    agent.cf.commander.send_stop_setpoint()
        else:
            if all([(agent.battery_test_passed and agent.extpos_test_passed) for agent in self.swarm_agent_list]):
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
        kp = 0.50
        ks = 0.1
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
        agent.standby_position[2] = self.manual_z
        agent.standby_yaw = self.manual_yaw

        if self.flightmode == 'position':
            agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                      agent.standby_position[2], agent.standby_yaw)
        elif self.flightmode == 'attitude':
            roll, pitch, yaw, thrust = agent.attitude_control_law(agent.standby_position, agent.standby_yaw)
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

    def takeoff_control_law(self, agent: Agent):
        if self.flightmode == 'position':
            agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                      agent.takeoff_position[2], agent.takeoff_yaw)
        elif self.flightmode == 'attitude':
            roll, pitch, yaw, thrust = agent.attitude_control_law(agent.takeoff_position, 0)
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

        d = distance([agent.extpos.x, agent.extpos.y,
                      agent.extpos.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Takeoff completed')
            agent.is_flying = True
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        if self.flightmode == 'position':
            agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                      agent.land_position[2], agent.land_yaw)
        elif self.flightmode == 'attitude':
            roll, pitch, yaw, thrust = agent.attitude_control_law(agent.land_position, agent.land_yaw)
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

        d = vertical_distance(agent.extpos.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Landing completed')
            agent.stop()

    def z_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt
                            for agt in self.swarm_agent_list if agt.state != 'Not flying' and agt != agent]
        kp = 1
        x_velocity = kp * (agent.z_consensus_xy_position[0] - agent.extpos.x)
        y_velocity = kp * (agent.z_consensus_xy_position[1] - agent.extpos.y)
        z_velocity = kp * (sum([agt.extpos.z - agent.extpos.z
                                for agt in in_flight_agents if agt.name in agent.z_consensus_connectivity]))
        agent.cf.commander.send_velocity_world_setpoint(x_velocity, y_velocity, z_velocity, 0)

    def standby_control_law(self, agent: Agent):
        if self.flightmode == 'position' or self.flightmode == 'velocity':
            agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
            objective = [agent.standby_position[0], agent.standby_position[1]]
            omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                     [agent.x_boundaries[1], agent.y_boundaries[1]]))
            vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                     objective, omega, self.xy_auto_avoid_d0)
            vz = agent.standby_position[2] - agent.extpos.z
            agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

        elif self.flightmode == 'attitude':
            roll, pitch, yaw, thrust = agent.attitude_control_law(agent.standby_position, agent.standby_yaw)
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)

    def wingman_control_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        leader_agent = [agt for agt in self.swarm_agent_list if agt.name == self.swarm_leader]
        objective = [leader_agent[0].extpos.x, leader_agent[0].extpos.y]
        omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                 [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.wingman_z - agent.extpos.z
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def back_to_initial_position_ctl_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.initial_position[0], agent.initial_position[1]]
        omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                 [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.takeoff_height - agent.extpos.z
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def land_on_air_base(self, agent):
        uav_handled = False
        position_to_reach = None
        for ab in self.air_base_list:
            if ab.handled_uav == agent:
                uav_handled, position_to_reach = ab.handle_uav(agent, 'Land')

        if not uav_handled:
            for ab in self.air_base_list:
                if ab.available:
                    uav_handled, position_to_reach = ab.handle_uav(agent, 'Land')
                    if uav_handled and position_to_reach is not None:
                        break

        if uav_handled and position_to_reach is not None:
            agents_to_avoid = [agt for agt in self.swarm_agent_list if (agt.name in agent.xy_auto_avoid_agents_list
                                                                        and not agt.is_flying)]
            if agent.name == 'cf2':
                print([agt.name for agt in agents_to_avoid])
            objective = [position_to_reach[0], position_to_reach[1]]
            omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                     [agent.x_boundaries[1], agent.y_boundaries[1]]))
            vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries,
                                                     agents_to_avoid,
                                                     objective, omega, self.xy_auto_avoid_d0)
            vz = position_to_reach[2] - agent.extpos.z

            yaw_rate = agent.yaw - position_to_reach[3]

            agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)
        else:
            agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
            objective = [agent.standby_position[0], agent.standby_position[1]]
            omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                     [agent.x_boundaries[1], agent.y_boundaries[1]]))
            vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                     objective, omega, self.xy_auto_avoid_d0)
            vz = agent.standby_position[2] - agent.extpos.z
            agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def sleep_on_air_base(self, agent: Agent):
        uav_found = False
        for ab in self.air_base_list:
            for slot in ab.slots:
                if slot.uav_parked == agent:
                    uav_found = True
                    phi = 0.0
                    theta = 0.0
                    psi = ab.yaw * np.pi / 180

                    qw = math.cos(phi) * math.cos(theta / 2) * math.cos(psi / 2) \
                         - math.sin(phi / 2) * math.sin(theta / 2) * math.sin(psi / 2)
                    qx = math.sin(phi / 2) * math.cos(theta / 2) * math.cos(psi / 2) \
                         + math.cos(phi / 2) * math.sin(theta / 2) * math.sin(psi / 2)
                    qy = math.sin(phi / 2) * math.cos(theta / 2) * math.sin(psi / 2) \
                         - math.cos(phi / 2) * math.sin(theta / 2) * math.cos(psi / 2)
                    qz = math.cos(phi / 2) * math.cos(theta / 2) * math.sin(psi / 2) \
                         + math.sin(phi / 2) * math.sin(theta / 2) * math.cos(psi / 2)

                    qn = math.sqrt((qw**2) + (qx**2) + (qy**2) + (qz**2))
                    qw = qw / qn
                    qx = qx / qn
                    qy = qy / qn
                    qz = qz / qn

                    agent.csv_logger.writerow([agent.name, agent.timestamp,
                                               agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                               agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                               agent.extpos.x, agent.extpos.y, agent.extpos.z,
                                               ab.yaw,
                                               0, 0, 0,
                                               0, 0, 0, 0])

                    agent.cf.extpos.send_extpose(agent.extpos.x, agent.extpos.y, agent.extpos.z, qx, qy, qz, qw)
        if not uav_found:
            print(' -- Warning -- :', agent.name, 'not found on any declared Air Base')
            agent.stop()

    def takeoff_from_air_base(self, agent):
        uav_found = False
        for ab in self.air_base_list:
            for slot in ab.slots:
                if slot.uav_parked == agent:
                    uav_found = True
                    uav_handled, position_to_reach = ab.handle_uav(agent, 'Takeoff')
                    if uav_handled and position_to_reach is not None:
                        agents_to_avoid = [agt for agt in self.swarm_agent_list if
                                           agt.name in agent.xy_auto_avoid_agents_list and not agt.is_flying]
                        objective = [position_to_reach[0], position_to_reach[1]]
                        omega = np.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                                 [agent.x_boundaries[1], agent.y_boundaries[1]]))
                        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries,
                                                                 agents_to_avoid,
                                                                 objective, omega, self.xy_auto_avoid_d0)
                        vz = position_to_reach[2] - agent.extpos.z
                        yaw_rate = agent.yaw - position_to_reach[3]
                        # yaw_rate = position_to_reach[3] - agent.yaw
                        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)
                        break
                    else:
                        agent.cf.commander.send_stop_setpoint()

        if not uav_found:
            print(' -- Warning -- :', agent.name, 'not found on any of the declared Air Bases')
            print('                 -> takeoff command access denied, switching back to Not flying mode')
            agent.stop()


def get_auto_avoid_velocity_command(agent: Agent, x_limits: [float] * 2, y_limits: [float] * 2,
                                    agents_to_avoid: List[Agent], objective: [float] * 2, omega: float, d0: float):
    vx = 0.0
    vy = 0.0

    # Borders
    if agent.extpos.x - x_limits[0] < 0:
        vx = vx + np.exp(-(agent.extpos.x - x_limits[0]))
    if agent.extpos.y - y_limits[0] < 0:
        vy = vy + np.exp(-(agent.extpos.y - y_limits[0]))
    if x_limits[1] - agent.extpos.x < 0:
        vx = vx - np.exp(-(x_limits[1] - agent.extpos.x))
    if y_limits[1] - agent.extpos.y < 0:
        vy = vy - np.exp(-(y_limits[1] - agent.extpos.y))

    # Agents to avoid
    kpo = 3
    kvo = 1
    for agt in agents_to_avoid:
        v = np.sqrt(agt.velocity[0]**2 + agt.velocity[1]**2)
        if v > 0.25:
            kv2 = (kvo * v)**2
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
    kpg = 2
    distance_to_objective = horizontal_distance([agent.extpos.x, agent.extpos.y],
                                                [objective[0],
                                                 objective[1]])

    d1 = np.pi / (2 * omega)
    vx = vx + ((objective[0] - agent.extpos.x) * kpg
               / (2 * d1 * np.sqrt((distance_to_objective + 0.001) / d1)))
    vy = vy + ((objective[1] - agent.extpos.y) * kpg
               / (2 * d1 * np.sqrt((distance_to_objective + 0.001) / d1)))

    return vx, vy


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
