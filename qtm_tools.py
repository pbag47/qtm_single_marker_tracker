import asyncio
import qtm
import numpy as np

from agent_class import Agent
from air_base_class import SquareAirBase
from qtm import QRTConnection
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List


async def connect_to_qtm(ip: str):
    connection: QRTConnection = await qtm.connect(ip)
    if connection is None:
        print(' ---- Warning ---- Error during QTM connection @', ip, '***')
    else:
        print('QTM connected @', ip)
    return connection


def frame_acquisition(connection: QRTConnection):
    frame: qtm.QRTPacket = asyncio.get_event_loop().run_until_complete(
        connection.get_current_frame(components=['3dnolabels']))
    timestamp = frame.timestamp * 10 ** -6
    headers, markers = frame.get_3d_markers_no_label()
    return headers, markers, timestamp


def initial_uav_detection(agents: List[Agent], air_bases: List[SquareAirBase], markers: List[RT3DMarkerPositionNoLabel],
                          timestamp: float):
    if len(agents) + 4 * len(air_bases) > len(markers):
        print(' -- Warning -- : Too few detected makers than expected, flight disabled')
        for agt in agents:
            agt.enabled = False
    elif len(agents) + 4 * len(air_bases) < len(markers):
        print(' -- Warning -- : Too many detected markers than expected, flight disabled')
        for agt in agents:
            agt.enabled = False

    for ab in air_bases:
        m_init_pos = [[ab.initial_position[0] + ab.markers_square_side_length / 2,
                       ab.initial_position[1] + ab.markers_square_side_length / 2,
                       ab.initial_position[2]],
                      [ab.initial_position[0] + ab.markers_square_side_length / 2,
                       ab.initial_position[1] - ab.markers_square_side_length / 2,
                       ab.initial_position[2]],
                      [ab.initial_position[0] - ab.markers_square_side_length / 2,
                       ab.initial_position[1] + ab.markers_square_side_length / 2,
                       ab.initial_position[2]],
                      [ab.initial_position[0] - ab.markers_square_side_length / 2,
                       ab.initial_position[1] - ab.markers_square_side_length / 2,
                       ab.initial_position[2]]]

        for i in range(len(ab.markers)):
            d = [distance_init_pos_to_marker(m_init_pos[i],
                                             [mk.x * 10 ** -3, mk.y * 10 ** -3, mk.z * 10 ** -3]) for mk in markers]
            try:
                min_d_index = d.index(min(d))
                ab.markers[i] = markers.pop(min_d_index)
                print('Air Base : marker found @', ab.markers[i])
            except ValueError:
                break

    for agt in agents:
        d = [distance_init_pos_to_marker(agt.initial_position,
                                         [mk.x * 10 ** -3, mk.y * 10 ** -3, mk.z * 10 ** -3]) for mk in markers]
        try:
            min_d_index = d.index(min(d))
            agt.update_extpos(markers.pop(min_d_index), timestamp)
            print(agt.name, 'found @', agt.extpos)
        except ValueError:
            break


def uav_tracking(agents: List[Agent], air_bases: List[SquareAirBase],
                 markers: List[RT3DMarkerPositionNoLabel], timestamp: float):
    markers_ids = [mk.id for mk in markers]

    for ab in air_bases:
        for i in range(len(ab.markers)):
            try:
                index = markers_ids.index(ab.markers[i].id)
                ab.markers[i] = markers.pop(index)
                del markers_ids[index]
            except ValueError:
                ab.invalid_6dof_count += 1
                if ab.invalid_6dof_count == 1:
                    print('Warning, AirBase lost')
                if ab.invalid_6dof_count == 10:
                    print('Warning : AirBase off camera for too long, switching its status to disabled')
                    ab.enabled = False
        ab.update_position()

    for agt in agents:
        if agt.setup_finished and agt.enabled:
            try:
                index = markers_ids.index(agt.extpos.id)
                agt.update_extpos(markers.pop(index), timestamp)
                agt.send_external_position()
                del markers_ids[index]
            except ValueError:
                agt.invalid_6dof_count += 1
                if agt.invalid_6dof_count == 1:
                    print('Warning :', agt.name, 'lost')
                if agt.invalid_6dof_count == 10:
                    print(' ---- Warning ----', agt.name, 'off camera for too long, switching the engines off')
                    agt.stop()

    for agt in agents:
        if agt.invalid_6dof_count > 0:
            d = [distance_between_markers(agt.extpos, mk) for mk in markers]
            try:
                if min(d) < 0.1:
                    min_d_index = d.index(min(d))
                    agt.update_extpos(markers.pop(min_d_index), timestamp)
                    agt.send_external_position()
                    agt.invalid_6dof_count = 0
                    print(agt.name, 'found @', agt.extpos)
            except ValueError:
                break


def distance_init_pos_to_marker(position_1: [float] * 3, position_2: [float] * 3):
    d = np.sqrt((position_1[0] - position_2[0]) ** 2
                + (position_1[1] - position_2[1]) ** 2
                + (position_1[2] - position_2[2]) ** 2)
    return d


def distance_between_markers(marker_1: RT3DMarkerPositionNoLabel, marker_2: RT3DMarkerPositionNoLabel):
    """
    Method that returns the distance (m) between marker_1 position (m) and
    marker_2 position (mm)
    """
    d = np.sqrt((marker_1.x - marker_2.x * 10 ** -3) ** 2
                + (marker_1.y - marker_2.y * 10 ** -3) ** 2
                + (marker_1.z - marker_2.z * 10 ** -3) ** 2)
    return d


async def disconnect_qtm(connection: QRTConnection):
    if connection is not None:
        await connection.stream_frames_stop()
        connection.disconnect()
        print('QTM disconnected')
    else:
        print('QTM connection already closed')


if __name__ == '__main__':
    qtm_ip_address = '192.168.0.1'
    qtm_connection = asyncio.get_event_loop().run_until_complete(connect_to_qtm(qtm_ip_address))
    h, m, t = frame_acquisition(qtm_connection)
    print('QTM packet received:')
    print('    Headers:', h)
    print('    Markers:', m)
    print('    Timestamp:', t)
    disconnect_qtm(qtm_connection)
