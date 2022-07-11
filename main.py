import asyncio
import cflib.crtp
import csv
import pynput.keyboard
import qtm_tools
import swarm_object_class

# from air_base_class import SquareAirBase
from joystick_class import Joystick
from qtm import QRTConnection
from qtm.packet import QRTPacket
from swarm_object_class import SwarmObject


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

        if key == pynput.keyboard.Key.shift:
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.takeoff()

        if key == pynput.keyboard.Key.ctrl:
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.land()


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
    qtm_tools.uav_tracking(SWARM_MANAGER.swarm_agent_list, SWARM_MANAGER.air_base_list, markers, timestamp)

    if PACKET_COUNT > -1:
        PACKET_COUNT = 0
        send_packet = True
    else:
        PACKET_COUNT = PACKET_COUNT + 1
        send_packet = False

    if send_packet:
        SWARM_MANAGER.flight_sequence()

    RUN_TRACKER = True


if __name__ == '__main__':
    # -- Flight parameters ------------------------------------------------------- #
    qtm_ip_address: str = '192.168.0.1'

    all_agents = [swarm_object_class.Agent('cf1', 'radio://0/83/2M/E7E7E7E701'),
                  swarm_object_class.Agent('cf2', 'radio://1/85/2M/E7E7E7E702'),
                  swarm_object_class.Agent('cf3', 'radio://1/90/2M/E7E7E7E703'),
                  swarm_object_class.Agent('cf4', 'radio://0/90/2M/E7E7E7E704')]

    all_agents[0].set_initial_position([0.5, 0.5, 0.0])
    all_agents[0].set_takeoff_height(0.4)
    all_agents[0].set_z_consensus_connectivity(['cf2', 'cf3', 'cf4'])  # ['cf2', 'cf3', 'cf4']
    all_agents[0].set_agents_to_avoid(['cf2', 'cf3', 'cf4'])

    all_agents[1].set_initial_position([0.5, -0.5, 0.0])
    all_agents[1].set_takeoff_height(0.60)
    all_agents[1].set_z_consensus_connectivity(['cf1', 'cf3', 'cf4'])  # ['cf1', 'cf3', 'cf4']
    all_agents[1].set_agents_to_avoid(['cf1', 'cf3', 'cf4'])

    all_agents[2].set_initial_position([-0.5, 0.5, 0.0])
    all_agents[2].set_takeoff_height(0.70)
    all_agents[2].set_z_consensus_connectivity(['cf1', 'cf2', 'cf4'])  # ['cf1', 'cf2', 'cf4']
    all_agents[2].set_agents_to_avoid(['cf1', 'cf2', 'cf4'])

    all_agents[3].set_initial_position([-0.5, -0.5, 0.0])
    all_agents[3].set_takeoff_height(0.80)
    all_agents[3].set_z_consensus_connectivity(['cf1', 'cf2', 'cf3'])  # ['cf1', 'cf2', 'cf3']
    all_agents[3].set_agents_to_avoid(['cf1', 'cf2', 'cf3'])

    # agents = []
    # agents = [all_agents[0]]
    # agents = [all_agents[0], all_agents[1]]
    # agents = [all_agents[3], all_agents[1], all_agents[2]]
    agents = [all_agents[0], all_agents[1], all_agents[2], all_agents[3]]

    air_bases = []
    # air_bases = [SquareAirBase()]
    # air_bases[0].initial_position = [0.0, -0.5, 0.15]

    # -- QTM connection and initial frame acquisition ---------------------------- #
    qtm_connection: QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))
    h, m, t = qtm_tools.frame_acquisition(qtm_connection)
    print(h.marker_count, 'markers found by QTM during initialization')
    qtm_tools.initial_uav_detection(agents, air_bases, m, t)

    cflib.crtp.init_drivers()

    SWARM_MANAGER = SwarmObject()
    js = Joystick(SWARM_MANAGER)
    for agent in agents:
        agent.connect_cf()
        SWARM_MANAGER.add_agent(agent)

    SWARM_MANAGER.air_base_list = air_bases

    SWARM_MANAGER.swarm_leader = 'cf1'
    SWARM_MANAGER.manual_flight_agents_list = ['cf1']

    SWARM_MANAGER.flightmode = 'position'

    # Boolean : True to save cf position in a csv file
    LOG_ENABLED = True
    RUN_TRACKER = True
    PACKET_COUNT = 0

    if LOG_ENABLED:
        file = open('logs.csv', 'w')
        WRITER = csv.writer(file)
        WRITER.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                         'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                         'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)',
                         'x_g (m)', 'y_g (m)', 'z_g (m)', 'yaw_g (°)',
                         'PID_zp', 'PID_zd', 'PID_zi',
                         'roll_c (°)', 'pitch_c (°)', 'yaw_c (°)', 'thrust_c (PWM)'])
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
