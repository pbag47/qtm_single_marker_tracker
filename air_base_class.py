from agent_class import Agent
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List, Union

import numpy as np


class SquareAirBase:
    def __init__(self):
        self.enabled = False
        self.available = True

        self.handled_uav: Union[None, Agent] = None
        self.handled_uav_slot: Union[None, Slot] = None

        self.yaw: float = 0.0  # (°)
        self.x: float = 0.0  # (m)
        self.y: float = 0.0  # (m)
        self.z: float = 0.0  # (m)
        self.initial_position: [float] * 3 = [0.0, 0.0, 0.0]  # Position of the center of the Air Base
        self.markers_square_side_length = 0.5  # (m)

        self.markers: List[RT3DMarkerPositionNoLabel] = [RT3DMarkerPositionNoLabel(0, 0, 0, None),
                                                         RT3DMarkerPositionNoLabel(0, 0, 0, None),
                                                         RT3DMarkerPositionNoLabel(0, 0, 0, None),
                                                         RT3DMarkerPositionNoLabel(0, 0, 0, None)]
        self.invalid_6dof_count: int = 0

        self.landing_zone_x_size = 0.5  # (m)
        self.landing_zone_y_size = 0.5  # (m)
        self.slot_x_size: float = 0.2  # (m)
        self.slot_y_size: float = 0.2  # (m)

        self.x_slots_number = int(self.landing_zone_x_size / self.slot_x_size)
        self.y_slots_number = int(self.landing_zone_y_size / self.slot_y_size)
        self.slots_number: int = self.x_slots_number * self.y_slots_number

        self.x_margin_length: float = ((self.landing_zone_x_size - (self.x_slots_number * self.slot_x_size))
                                       / (self.x_slots_number + 1))
        self.y_margin_length: float = ((self.landing_zone_y_size - (self.y_slots_number * self.slot_y_size))
                                       / (self.y_slots_number + 1))

        self.slot_matrix: List[List[Slot]] = []
        self.slots: List[Slot] = []
        for ix in range(self.x_slots_number):
            self.slot_matrix.append([])
            for iy in range(self.y_slots_number):
                self.slot_matrix[ix].append(Slot())
                self.slots.append(self.slot_matrix[ix][iy])

        self.enabled = True

    def update_position(self):
        x31 = self.markers[0].x - self.markers[2].x
        y31 = self.markers[0].y - self.markers[2].y

        x42 = self.markers[1].x - self.markers[3].x
        y42 = self.markers[1].y - self.markers[3].y

        dx = (x31 + x42) / 2
        dy = (y31 + y42) / 2

        try:
            if dx > 0:  # -> -90 < yaw < 90
                if dy > 0:  # -> yaw > 0
                    self.yaw = (np.arctan(dy / dx)) * 180 / np.pi
                else:
                    self.yaw = (np.arctan(dy / dx)) * 180 / np.pi
            else:
                if dy > 0:
                    self.yaw = (np.arctan(dy / dx) + np.pi) * 180 / np.pi
                else:
                    self.yaw = (np.arctan(dy / dx) - np.pi) * 180 / np.pi

        except ZeroDivisionError:
            print('Warning : Yaw, attempted to divide by zero')

        self.x = float(np.mean([mk.x / 1000 for mk in self.markers]))
        self.y = float(np.mean([mk.y / 1000 for mk in self.markers]))
        self.z = float(np.mean([mk.z / 1000 for mk in self.markers]))

        self.available = False

        for ix in range(self.x_slots_number):
            for iy in range(self.y_slots_number):
                self.slot_matrix[ix][iy].x = (self.x
                                              + (- self.landing_zone_x_size / 2
                                                 + ((ix + 1) * self.x_margin_length)
                                                 + ((ix + (1 / 2)) * self.slot_x_size)) * np.cos(self.yaw * np.pi / 180)
                                              - (- self.landing_zone_y_size / 2
                                                 + ((iy + 1) * self.y_margin_length)
                                                 + ((iy + (1 / 2)) * self.slot_y_size))
                                              * np.sin(self.yaw * np.pi / 180))

                self.slot_matrix[ix][iy].y = (self.y
                                              + (- self.landing_zone_x_size / 2
                                                 + ((ix + 1) * self.x_margin_length)
                                                 + ((ix + (1 / 2)) * self.slot_x_size)) * np.sin(self.yaw * np.pi / 180)
                                              + (- self.landing_zone_y_size / 2
                                                 + ((iy + 1) * self.y_margin_length)
                                                 + ((iy + (1 / 2)) * self.slot_y_size))
                                              * np.cos(self.yaw * np.pi / 180))

                self.slot_matrix[ix][iy].z = self.z

                if self.slot_matrix[ix][iy].available:
                    self.available = True

    def handle_uav(self, agent: Agent, request: str):
        position_to_reach = None

        if self.handled_uav is None:
            self.handled_uav = agent
            print('Air Base : Handling', agent.name, 'with request |', request, '|')

        if self.handled_uav == agent:
            uav_handled = True
            if request == 'Land':
                if self.handled_uav_slot is None:
                    for slot in self.slots:
                        if slot.available:
                            self.handled_uav_slot = slot
                            self.handled_uav_slot.available = False
                            break
                    position_to_reach = [self.handled_uav_slot.x, self.handled_uav_slot.y,
                                         self.handled_uav_slot.z + 0.5, self.yaw]
                elif not self.handled_uav_slot.stop_point_reached:
                    position_to_reach = [self.handled_uav_slot.x, self.handled_uav_slot.y,
                                         self.handled_uav_slot.z + 0.5, self.yaw]
                    distance_to_stop_point = np.sqrt((agent.extpos.x - self.handled_uav_slot.x) ** 2
                                                     + (agent.extpos.y - self.handled_uav_slot.y) ** 2
                                                     + (agent.extpos.z - (self.handled_uav_slot.z + 0.5)) ** 2)
                    if (distance_to_stop_point <= 0.05
                            and np.sqrt(
                                agent.velocity[0] ** 2 + agent.velocity[1] ** 2 + agent.velocity[2] ** 2) <= 0.02):
                        self.handled_uav_slot.stop_point_reached = True
                else:
                    position_to_reach = [self.handled_uav_slot.x, self.handled_uav_slot.y,
                                         self.handled_uav_slot.z, self.yaw]
                    vertical_distance_to_slot = np.sqrt((agent.extpos.z - self.handled_uav_slot.z) ** 2)
                    if vertical_distance_to_slot <= 0.15:
                        agent.sleep_on_air_base()
                        agent.cf.commander.send_stop_setpoint()
                        self.handled_uav_slot.uav_parked = agent
                        self.handled_uav = None
                        self.handled_uav_slot = None
                        print('Air Base : |', request, '| procedure for', agent.name, 'finished')

            elif request == 'Takeoff':
                if self.handled_uav_slot is None:
                    for slot in self.slots:
                        if slot.uav_parked == agent:
                            self.handled_uav_slot = slot
                            break
                    position_to_reach = [self.handled_uav_slot.x, self.handled_uav_slot.y,
                                         self.handled_uav_slot.z + 0.5, self.yaw]
                else:
                    position_to_reach = [self.handled_uav_slot.x, self.handled_uav_slot.y,
                                         self.handled_uav_slot.z + 0.5, self.yaw]
                    distance_to_stop_point = np.sqrt((agent.extpos.x - self.handled_uav_slot.x) ** 2
                                                     + (agent.extpos.y - self.handled_uav_slot.y) ** 2
                                                     + (agent.extpos.z - (self.handled_uav_slot.z + 0.5)) ** 2)
                    if (distance_to_stop_point <= 0.05
                            and np.sqrt(
                                agent.velocity[0] ** 2 + agent.velocity[1] ** 2 + agent.velocity[2] ** 2) <= 0.02):
                        agent.is_flying = True
                        agent.back_to_initial_position()
                        self.handled_uav_slot.uav_parked = None
                        self.handled_uav_slot.available = True
                        self.handled_uav = None
                        self.handled_uav_slot = None
                        print('Air Base : |', request, '| procedure for', agent.name, 'finished')
        else:
            uav_handled = False

        if position_to_reach is not None:
            agent.csv_logger.writerow([agent.name, agent.timestamp,
                                       agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                       agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                       position_to_reach[0], position_to_reach[1],
                                       position_to_reach[2], position_to_reach[3],
                                       0, 0, 0,
                                       0, 0, 0, 0])
        return uav_handled, position_to_reach


class Slot:
    def __init__(self):
        self.x: float = 0.0  # (m)
        self.y: float = 0.0  # (m)
        self.z: float = 0.0  # (m)

        self.stop_point_reached: bool = False
        self.uav_parked: Union[None, Agent] = None
        self.available: bool = True


if __name__ == '__main__':
    A = SquareAirBase()
    print(A.slots_number)
