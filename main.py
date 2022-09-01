import asyncio
import cflib.crtp
import csv
import pynput.keyboard
import qtm_tools
import swarm_object_class

from joystick_class import Joystick
from network_communication_class import NetworkCommunication
from qtm import QRTConnection
from qtm.packet import QRTPacket
from swarm_object_class import SwarmObject
from typing import List, Tuple


async def start_qtm_streaming(connection: QRTConnection):
    """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
     This method is made to run forever in an asyncio event loop """
    print('QTM streaming started')
    await connection.stream_frames(components=['3dnolabels'], on_packet=packet_reception_callback)


async def keyboard_handler():
    global SWARM_MANAGER
    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            print('Disconnecting...')
            for agt in SWARM_MANAGER.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            asyncio.get_event_loop().stop()


def detect_keyboard_input():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def on_press_callback(key):
        try:
            loop.call_soon_threadsafe(queue.put_nowait, key.char)
        except AttributeError:
            loop.call_soon_threadsafe(queue.put_nowait, key)

    pynput.keyboard.Listener(on_press=on_press_callback).start()
    return queue


def packet_reception_callback(packet: QRTPacket):
    global SWARM_MANAGER
    global RUN_TRACKER
    global PACKET_COUNT

    if not RUN_TRACKER:
        print(' ---- Warning ---- Callback execution interrupted by new QTM packet')
        print('                   -> There might be an error occurring during callback execution')
        print('                   -> Or, the sequence might require too much computing load')
        for agents_to_stop in SWARM_MANAGER.swarm_agent_list:
            agents_to_stop.stop()
    RUN_TRACKER = False

    timestamp = packet.timestamp * 10**-6
    headers, markers = packet.get_3d_markers_no_label()
    qtm_tools.uav_tracking(SWARM_MANAGER.swarm_agent_list, markers, timestamp)

    # if PACKET_COUNT > -1:
    #     PACKET_COUNT = 0
    #     send_packet = True
    # else:
    #     PACKET_COUNT = PACKET_COUNT + 1
    #     send_packet = False

    # if send_packet:
    SWARM_MANAGER.flight_sequence()

    RUN_TRACKER = True


if __name__ == '__main__':
    # -- Flight parameters ------------------------------------------------------- #
    qtm_ip_address: str = '192.168.0.1'

    this_PC_attributes: Tuple[str, int] = ('192.168.0.102', 4444)
    other_PCs_attributes: List[Tuple[str, int]] = [('192.168.0.100', 4444)]

    all_agents = [swarm_object_class.Agent('cf1', 'radio://0/81/2M/E7E7E7E701'),
                  swarm_object_class.Agent('cf2', 'radio://0/82/2M/E7E7E7E702'),
                  swarm_object_class.Agent('cf3', 'radio://1/83/2M/E7E7E7E703'),
                  swarm_object_class.Agent('cf4', 'radio://1/84/2M/E7E7E7E704'),
                  swarm_object_class.Agent('cf5', 'radio://2/85/2M/E7E7E7E705'),
                  swarm_object_class.Agent('cf6', 'radio://2/86/2M/E7E7E7E706'),
                  swarm_object_class.Agent('cf7', 'radio://3/87/2M/E7E7E7E707'),
                  swarm_object_class.Agent('cf8', 'radio://3/88/2M/E7E7E7E708'),
                  swarm_object_class.Agent('cf9', 'radio://4/89/2M/E7E7E7E709'),
                  swarm_object_class.Agent('cf10', 'radio://4/90/2M/E7E7E7E710')]

    all_agents[0].set_initial_position([1.0, 1.0, 0.0])
    all_agents[0].set_takeoff_height(0.50)
    all_agents[0].set_z_consensus_connectivity(['cf2', 'cf3', 'cf4'])  # ['cf2', 'cf3', 'cf4']
    all_agents[0].set_agents_to_avoid(['cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[1].set_initial_position([1.0, -1.0, 0.0])
    all_agents[1].set_takeoff_height(0.55)
    all_agents[1].set_z_consensus_connectivity(['cf1', 'cf3', 'cf4'])  # ['cf1', 'cf3', 'cf4']
    all_agents[1].set_agents_to_avoid(['cf1', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[2].set_initial_position([-1.0, 1.0, 0.0])
    all_agents[2].set_takeoff_height(0.55)
    all_agents[2].set_z_consensus_connectivity(['cf1', 'cf2', 'cf4'])  # ['cf1', 'cf2', 'cf4']
    all_agents[2].set_agents_to_avoid(['cf1', 'cf2', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[3].set_initial_position([-1.0, -1.0, 0.0])
    all_agents[3].set_takeoff_height(0.60)
    all_agents[3].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[3].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf5', 'cf6', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[4].set_initial_position([0.0, 0.6, 0.0])
    all_agents[4].set_takeoff_height(0.65)
    all_agents[4].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[4].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf6', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[5].set_initial_position([0.0, -0.6, 0.0])
    all_agents[5].set_takeoff_height(0.65)
    all_agents[5].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[5].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf7', 'cf8', 'cf9', 'cf10'])

    all_agents[6].set_initial_position([-0.4, 0.0, 0.0])
    all_agents[6].set_takeoff_height(0.60)
    all_agents[6].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[6].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf8', 'cf9', 'cf10'])

    all_agents[7].set_initial_position([-1.0, 1.0, 0.0])
    all_agents[7].set_takeoff_height(0.55)
    all_agents[7].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[7].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf9', 'cf10'])

    all_agents[8].set_initial_position([-1.0, -1.0, 0.0])
    all_agents[8].set_takeoff_height(0.55)
    all_agents[8].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[8].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8', 'cf10'])

    all_agents[9].set_initial_position([-1.25, 0.0, 0.0])
    all_agents[9].set_takeoff_height(0.50)
    all_agents[9].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[9].set_agents_to_avoid(['cf1', 'cf2', 'cf3', 'cf4', 'cf5', 'cf6', 'cf7', 'cf8', 'cf9'])

    # agents = []
    # agents = [all_agents[0]]
    # agents = [all_agents[0], all_agents[1]]
    # agents = [all_agents[0], all_agents[1], all_agents[2]]
    agents = [all_agents[0], all_agents[1], all_agents[2], all_agents[3], all_agents[4], all_agents[5]]
    # agents = [all_agents[1], all_agents[2], all_agents[3], all_agents[4], all_agents[5]]
    # agents = all_agents

    # -- QTM connection and initial frame acquisition ---------------------------- #
    qtm_connection: QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))
    h, m, t = qtm_tools.frame_acquisition(qtm_connection)
    print(h.marker_count, 'markers found by QTM during initialization')
    qtm_tools.initial_uav_detection(agents, m, t)

    cflib.crtp.init_drivers()

    SWARM_MANAGER = SwarmObject()
    communication_tool = NetworkCommunication(this_PC_attributes, other_PCs_attributes)
    joystick_connected = True
    js = Joystick(SWARM_MANAGER, communication_tool, joystick_connected)
    for agent in agents:
        agent.connect_cf()
        SWARM_MANAGER.add_agent(agent)

    SWARM_MANAGER.swarm_leader = 'cf1'
    SWARM_MANAGER.manual_flight_agents_list = ['cf1']

    # Boolean : True to save cf position in a csv file
    LOG_ENABLED = True
    RUN_TRACKER = True
    PACKET_COUNT = 0

    if LOG_ENABLED:
        file = open('logs.csv', 'w')
        WRITER = csv.writer(file)
        WRITER.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                         'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                         'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)'])
        for agent in agents:
            agent.csv_logger = WRITER
    else:
        file = None
        WRITER = None

    asyncio.ensure_future(start_qtm_streaming(qtm_connection))
    asyncio.ensure_future(keyboard_handler())
    asyncio.get_event_loop().run_forever()

    # Disconnects the Crazyflies, stops the QTM stream and disconnects QTM
    for agent in agents:
        agent.stop()
        agent.enabled = False
        agent.cf.close_link()
    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(qtm_connection))

    if LOG_ENABLED:
        file.close()
