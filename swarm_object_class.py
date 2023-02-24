import numpy
import logging

from agent_class import Agent
from typing import List, Union


logger = logging.getLogger(__name__)


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
        logger.info(agent.name + ' added to the swarm')

    def remove_agent(self, agent: Agent):
        try:
            name_index = self.agent_name_list.index(agent.name)
            _ = self.agent_name_list.pop(name_index)
            __ = self.swarm_agent_list.pop(name_index)
            logger.warning(agent.name + ' has been removed from the swarm')
            if self.swarm_leader == agent.name:
                self.swarm_leader = None
                logger.warning('The leader has been removed. No leader assigned for the swarm')
        except ValueError:
            logger.error('Unable to remove ' + agent.name + ' from the swarm')

    def assign_swarm_leader(self, cf_name: str):
        self.swarm_leader = cf_name
        logger.info(cf_name + ' assigned as the swarm leader')

    def assign_target(self, cf_name: str):
        self.target = cf_name
        logger.info(cf_name + ' assigned as the target in pursuit-mode')

    def assign_chaser(self, cf_name: str):
        self.chaser = cf_name
        logger.info(cf_name + ' assigned as the chaser in pursuit-mode')

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

        elif all([(agent.battery_test_passed and agent.position_test_passed) for agent in self.swarm_agent_list]):
            logger.info('Swarm ready')
            self.ready_to_fly = True
            print('Crazyflie connection recap :')
            for agent in self.swarm_agent_list:
                agent.setup_finished = True
                print('    -', agent.name, ':')
                if agent.position.x >= 0:
                    print('        x = ', round(agent.position.x, 3), 'm')
                else:
                    print('        x =', round(agent.position.x, 3), 'm')
                if agent.position.y >= 0:
                    print('        y = ', round(agent.position.y, 3), 'm')
                else:
                    print('        y =', round(agent.position.y, 3), 'm')
                if agent.position.z >= 0:
                    print('        z = ', round(agent.position.z, 3), 'm')
                else:
                    print('        z =', round(agent.position.z, 3), 'm')
                print('        Battery level =', round(agent.initial_battery_level), '%')
                if agent.enabled:
                    print('        Flight enabled')
                else:
                    print('        -- Warning -- : Flight disabled')

    def manual_control_law(self, agent: Agent):
        kp = 0.5
        ks = 0.10

        if self.manual_x >= 0:
            agent.standby_position[0] = agent.position.x - kp * numpy.sqrt(self.manual_x)
        else:
            agent.standby_position[0] = agent.position.x + kp * numpy.sqrt(-self.manual_x)

        if self.manual_y >= 0:
            agent.standby_position[1] = agent.position.y - kp * numpy.sqrt(self.manual_y)
        else:
            agent.standby_position[1] = agent.position.y + kp * numpy.sqrt(-self.manual_y)

        if agent.standby_position[0] < agent.x_boundaries[0] + ks:
            agent.standby_position[0] = agent.x_boundaries[0] + ks
        if agent.standby_position[0] > agent.x_boundaries[1] - ks:
            agent.standby_position[0] = agent.x_boundaries[1] - ks
        if agent.standby_position[1] < agent.y_boundaries[0] + ks:
            agent.standby_position[1] = agent.y_boundaries[0] + ks
        if agent.standby_position[1] > agent.y_boundaries[1] - ks:
            agent.standby_position[1] = agent.y_boundaries[1] - ks

        agent.standby_position[2] = self.manual_z * (0.90 * agent.z_boundaries[1])
        if agent.standby_position[2] > agent.z_boundaries[1] - ks:
            agent.standby_position[2] = agent.z_boundaries[1] - ks
        agent.standby_yaw = self.manual_yaw
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = distance([agent.position.x, agent.position.y,
                      agent.position.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' takeoff completed')
            agent.is_flying = True
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   'None', 'None', 'None', 'None'])
        d = vertical_distance(agent.position.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' landing completed')
            agent.stop()
            self.remove_agent(agent)

    def xy_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt for agt in self.swarm_agent_list if agt.state != 'Not flying']
        roll, pitch = get_xy_consensus_attitude(agent, in_flight_agents)
        thrust = thrust_control_law(agent, agent.takeoff_height)
        yaw_rate = yaw_rate_control_law(agent, 0)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   'None', 'None', 'None',
                                   roll, pitch, yaw_rate, thrust])
        agent.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)

    def z_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt
                            for agt in self.swarm_agent_list if agt.state != 'Not flying' and agt != agent]
        kp = 1
        vx = kp * (agent.z_consensus_xy_position[0] - agent.position.x)
        vy = kp * (agent.z_consensus_xy_position[1] - agent.position.y)
        vz = kp * (sum([agt.position.z - agent.position.z
                        for agt in in_flight_agents if agt.name in agent.consensus_connectivity]))
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)

    def standby_control_law(self, agent: Agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.standby_position[0], agent.standby_position[1]]
        omega = numpy.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                    [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.standby_position[2] - agent.position.z
        yaw_rate = yaw_rate_control_law(agent, 0)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def wingman_control_law(self, agent):
        if not self.swarm_leader:
            logger.error('No swarm leader assigned, unable to switch to the Wingman control mode')
            agent.standby()
        else:
            agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
            leader_agent = [agt for agt in self.swarm_agent_list if agt.name == self.swarm_leader]
            objective = [leader_agent[0].position.x, leader_agent[0].position.y]
            omega = numpy.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                        [agent.x_boundaries[1], agent.y_boundaries[1]]))
            vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                     objective, omega, self.xy_auto_avoid_d0)
            vz = agent.wingman_z - agent.position.z
            yaw_rate = yaw_rate_control_law(agent, leader_agent[0].yaw)
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                       agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                       vx, vy, vz,
                                       'None', 'None', 'None', 'None'])
            agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def back_to_initial_position_ctl_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_agents_list]
        objective = [agent.initial_position[0], agent.initial_position[1]]
        omega = numpy.pi / (2 * horizontal_distance([agent.x_boundaries[0], agent.y_boundaries[0]],
                                                    [agent.x_boundaries[1], agent.y_boundaries[1]]))
        vx, vy = get_auto_avoid_velocity_command(agent, agent.x_boundaries, agent.y_boundaries, agents_to_avoid,
                                                 objective, omega, self.xy_auto_avoid_d0)
        vz = agent.takeoff_height - agent.position.z
        yaw_rate = yaw_rate_control_law(agent, agent.back_to_init_yaw)
        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   vx, vy, vz,
                                   'None', 'None', 'None', 'None'])
        agent.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)

    def pursuit_control_law(self, agent: Agent):
        if agent.name == self.target:
            kp = 0.30
            ks = 0.20
            if self.manual_x >= 0:
                agent.standby_position[0] = agent.position.x - kp * numpy.sqrt(self.manual_x)
                if agent.standby_position[0] < agent.x_boundaries[0] + ks:
                    agent.standby_position[0] = agent.x_boundaries[0] + ks
            else:
                agent.standby_position[0] = agent.position.x + kp * numpy.sqrt(-self.manual_x)
                if agent.standby_position[0] > agent.x_boundaries[1] - ks:
                    agent.standby_position[0] = agent.x_boundaries[1] - ks
            if self.manual_y >= 0:
                agent.standby_position[1] = agent.position.y - kp * numpy.sqrt(self.manual_y)
                if agent.standby_position[1] < agent.y_boundaries[0] + ks:
                    agent.standby_position[1] = agent.y_boundaries[0] + ks
            else:
                agent.standby_position[1] = agent.position.y + kp * numpy.sqrt(-self.manual_y)
                if agent.standby_position[1] > agent.y_boundaries[1] - ks:
                    agent.standby_position[1] = agent.y_boundaries[1] - ks
            agent.standby_position[2] = 0.25
            agent.standby_yaw = 0
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                       agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                       'None', 'None', 'None',
                                       'None', 'None', 'None', 'None'])
            agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                      agent.standby_position[2], agent.standby_yaw)
        elif agent.name == self.chaser:
            target = [agt for agt in self.swarm_agent_list if agt.name == self.target]
            d = horizontal_distance([agent.position.x, agent.position.y],
                                    [target[0].position.x, target[0].position.y])  # (m)
            if d <= 0.1:
                logger.critical('Impact')
                target[0].land()
                agent.standby()
            else:
                # -- Roll control law -- #
                vy = 0          # (m/s)
                kp = 1
                measured_vy = (- agent.velocity.x * numpy.sin(agent.yaw * numpy.pi / 180)
                               + agent.velocity.y * numpy.cos(agent.yaw * numpy.pi / 180))   # (m/s)
                roll = - round(kp * (vy - measured_vy) * 180 / numpy.pi)                   # (°)
                max_roll = 30  # (°)
                if roll > max_roll:
                    roll = max_roll
                elif roll < - max_roll:
                    roll = - max_roll

                # -- Pitch control law -- #
                vx = 1       # (m/s)
                kp = 1
                measured_vx = (agent.velocity.x * numpy.cos(agent.yaw * numpy.pi / 180)
                               + agent.velocity.y * numpy.sin(agent.yaw * numpy.pi / 180))   # (m/s)
                pitch = round(kp * (vx - measured_vx) * 180 / numpy.pi)                    # (°)
                max_pitch = 30  # (°)
                if pitch > max_pitch:
                    pitch = max_pitch
                elif pitch < - max_pitch:
                    pitch = - max_pitch

                # -- Yaw rate control law -- #
                closing_speed_x = target[0].velocity.x - agent.velocity.x
                closing_speed_y = target[0].velocity.y - agent.velocity.y
                closing_speed = numpy.sqrt(closing_speed_x ** 2 + closing_speed_y ** 2)

                a = 27
                eta = numpy.arctan2(target[0].position.y - agent.position.y,
                                    target[0].position.x - agent.position.x)               # (rad)
                yaw = (agent.yaw * numpy.pi / 180) + (a * closing_speed * (eta - self.pursuit_eta))    # (rad)
                self.pursuit_eta = eta                                              # (rad)
                yaw_rate = yaw_rate_control_law(agent, yaw * 180 / numpy.pi)           # (°/s)

                # -- Thrust control law -- #
                z = 0.5         # (m)
                thrust = thrust_control_law(agent, z)   # (PWM)

                # -- Volume borders security check -- #
                #       Interrupts the chase to keep the UAV within the flight volume boundaries
                ks = 0.30
                if (agent.position.x < agent.x_boundaries[0] + ks
                        or agent.position.x > agent.x_boundaries[1] - ks
                        or agent.position.y < agent.y_boundaries[0] + ks
                        or agent.position.y > agent.y_boundaries[1] - ks):
                    x = 6.5 * agent.position.x / 8        # (m)
                    y = 6.5 * agent.position.y / 8        # (m)
                    yaw = (180 / numpy.pi) * (numpy.pi + numpy.arctan2(agent.position.y, agent.position.x))    # (°)
                    agent.cf.commander.send_position_setpoint(x, y, z, yaw)
                    agent.csv_logger.writerow([agent.name, agent.timestamp,
                                               agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                               agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                               'None', 'None', 'None',
                                               'None', 'None', 'None', 'None'])
                else:
                    agent.csv_logger.writerow([agent.name, agent.timestamp,
                                               agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                               agent.velocity.x, agent.velocity.y, agent.velocity.z,
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
        v = numpy.sqrt(agt.velocity.x ** 2 + agt.velocity.y ** 2)
        if v > 0.25:
            kv2 = (kvo * v) ** 2
            xb = agt.position.x + kvo * agt.velocity.x
            yb = agt.position.y + kvo * agt.velocity.y
            xc = agt.position.x + (((xb - agt.position.x) / (kv2 + 0.001))
                                   * ((xb - agt.position.x) * (agent.position.x - agt.position.x)
                                      + (yb - agt.position.y) * (agent.position.y - agt.position.y)))
            yc = agt.position.y + ((xb - agt.position.x) * (agent.position.x - agt.position.x)
                                   + (yb - agt.position.y) * (agent.position.y - agt.position.y)) / (kv2 + 0.001)

            if ((agt.position.x < xc < xb or xb < xc < agt.position.x)
                    and (agt.position.y < yc < yb or yb < yc < agt.position.y)):
                d = horizontal_distance([agent.position.x, agent.position.y], [xc, yc])
                if d <= d0:
                    vx = vx - (((xc - agent.position.x) / (d + 0.001))
                               * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
                    vy = vy - (((yc - agent.position.y) / (d + 0.001))
                               * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
            else:
                da = horizontal_distance([agent.position.x, agent.position.y], [agt.position.x, agt.position.y])
                db = horizontal_distance([agent.position.x, agent.position.y], [xb, yb])
                if da < db and da <= d0:
                    vx = vx - (((agt.position.x - agent.position.x) / (da + 0.001))
                               * kpo * (numpy.exp(-da) - numpy.exp(-d0)))
                    vy = vy - (((agt.position.y - agent.position.y) / (da + 0.001))
                               * kpo * (numpy.exp(-da) - numpy.exp(-d0)))
                elif db < da and db < d0:
                    vx = vx - (((xb - agent.position.x) / (db + 0.001))
                               * kpo * (numpy.exp(-db) - numpy.exp(-d0)))
                    vy = vy - (((yb - agent.position.y) / (db + 0.001))
                               * kpo * (numpy.exp(-db) - numpy.exp(-d0)))
        else:
            d = horizontal_distance([agent.position.x, agent.position.y], [agt.position.x, agt.position.y])
            if d <= d0:
                vx = vx - (((agt.position.x - agent.position.x) / (d + 0.001))
                           * kpo * (numpy.exp(-d) - numpy.exp(-d0)))
                vy = vy - (((agt.position.y - agent.position.y) / (d + 0.001))
                           * kpo * (numpy.exp(-d) - numpy.exp(-d0)))

    # Objective
    kpg = 1
    distance_to_objective = horizontal_distance([agent.position.x, agent.position.y],
                                                [objective[0],
                                                 objective[1]])
    d1 = numpy.pi / (2 * omega)
    vx = vx + ((objective[0] - agent.position.x) * kpg
               / (2 * d1 * numpy.sqrt((distance_to_objective + 0.001) / d1)))
    vy = vy + ((objective[1] - agent.position.y) * kpg
               / (2 * d1 * numpy.sqrt((distance_to_objective + 0.001) / d1)))

    # Borders
    x_min = x_limits[0] + 0.2 * (x_limits[1] - x_limits[0])
    y_min = y_limits[0] + 0.2 * (y_limits[1] - y_limits[0])
    x_max = x_limits[1] - 0.2 * (x_limits[1] - x_limits[0])
    y_max = y_limits[1] - 0.2 * (y_limits[1] - y_limits[0])
    if agent.position.x < x_min and vx < 0:
        vx = 0
    if agent.position.y < y_min and vy < 0:
        vy = 0
    if agent.position.x > x_max and vx > 0:
        vx = 0
    if agent.position.y > y_max and vy > 0:
        vy = 0

    return vx, vy


def get_xy_consensus_attitude(agent: Agent, agents_list: List[Agent]):
    yaw = agent.yaw * numpy.pi / 180  # Convert from degrees to radians

    r = agent.xy_consensus_offset[0]  # (m)
    rho = agent.xy_consensus_offset[1]  # (m)

    connected_agents = [agt for agt in agents_list
                        if agt.name in agent.consensus_connectivity and agt != agent]

    kp = 1
    xi = 1
    kd = 0.2

    xn_errors_list = []
    yn_errors_list = []
    vxn_errors_list = []
    vyn_errors_list = []
    for agt in connected_agents:
        x_error = agt.position.x - agent.position.x
        y_error = agt.position.y - agent.position.y

        xn_error = x_error * numpy.cos(yaw) + y_error * numpy.sin(yaw)
        yn_error = - x_error * numpy.sin(yaw) + y_error * numpy.cos(yaw)
        xn_errors_list.append(xn_error)
        yn_errors_list.append(yn_error)

        vx_error = agent.velocity.x - agt.velocity.x
        vy_error = agent.velocity.y - agt.velocity.y

        vxn_error = vx_error * numpy.cos(yaw) + vy_error * numpy.sin(yaw)
        vyn_error = - vx_error * numpy.sin(yaw) + vy_error * numpy.cos(yaw)
        vxn_errors_list.append(vxn_error)
        vyn_errors_list.append(vyn_error)

    axn = kd * (kp * (sum(xn_errors_list) + r) - xi * sum(vxn_errors_list))
    ayn = kd * (kp * (sum(yn_errors_list) + rho) - xi * sum(vyn_errors_list))

    roll = - ayn  # (rad)
    pitch = axn  # (rad)

    pitch = pitch * 180 / numpy.pi  # Convert from radians to degrees
    roll = roll * 180 / numpy.pi  # Convert from radians to degrees

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

    z_error = targeted_z - agent.position.z
    pz = z_kp * z_error
    iz = agent.previous_iz + z_ki * z_error * agent.delta_t
    dz = - z_kd * agent.velocity.z
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

    yaw_kp = 3
    targeted_yaw = targeted_yaw * numpy.pi / 180  # (° -> rad)
    targeted_yaw = targeted_yaw % (2 * numpy.pi)
    if targeted_yaw > numpy.pi:
        targeted_yaw = targeted_yaw - (2 * numpy.pi)

    measured_yaw = agent.yaw * numpy.pi / 180
    measured_yaw = measured_yaw % (2 * numpy.pi)
    if measured_yaw > numpy.pi:
        measured_yaw = measured_yaw - (2 * numpy.pi)

    yaw_error_0 = targeted_yaw - measured_yaw
    yaw_error_1 = targeted_yaw - measured_yaw + 2 * numpy.pi
    yaw_error_2 = targeted_yaw - measured_yaw - 2 * numpy.pi

    if abs(yaw_error_1) < abs(yaw_error_0) and abs(yaw_error_1) < abs(yaw_error_2):
        yaw_error = yaw_error_1
    elif abs(yaw_error_2) < abs(yaw_error_0) and abs(yaw_error_2) < abs(yaw_error_1):
        yaw_error = yaw_error_2
    else:
        yaw_error = yaw_error_0

    yaw_rate = - round(yaw_kp * yaw_error * 180 / numpy.pi)  # (°/s)

    max_yaw_rate = 180  # (°/s)
    if yaw_rate > max_yaw_rate:
        yaw_rate = max_yaw_rate
    elif yaw_rate < -max_yaw_rate:
        yaw_rate = -max_yaw_rate
    return yaw_rate


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = numpy.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                   + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                   + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = numpy.sqrt((position_1_z - position_2_z) ** 2)
    return vd


def horizontal_distance(position_1_xy: List[float], position_2_xy: List[float]):
    hd = numpy.sqrt((position_1_xy[0] - position_2_xy[0]) ** 2
                    + (position_1_xy[1] - position_2_xy[1]) ** 2)
    return hd
