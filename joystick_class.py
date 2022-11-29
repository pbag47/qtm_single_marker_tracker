import array
import joystick_map
import os
import struct

from fcntl import ioctl
from swarm_object_class import SwarmObject
from network_communication_class import NetworkCommunication

from threading import Thread
from typing import Union


class Joystick:
    def __init__(self, swarm_object: Union[SwarmObject, None] = None,
                 network_communication: Union[NetworkCommunication, None] = None,
                 js_connected: bool = True):
        self.swarm = swarm_object
        self.network_comm = network_communication
        self.joystick_connected = js_connected
        self.stopped = False

        if not self.joystick_connected:
            self.packet_rec_thread = Thread(target=self.await_packet)
            self.packet_rec_thread.start()
        else:
            print('Joystick initialization:')
            print('    Available devices:', [('/dev/input/%s' % fn)
                                             for fn in os.listdir('/dev/input') if fn.startswith('js')])

            # Buttons and axes states initialization
            self.axis_states = {}
            self.button_states = {}

            self.axis_map = []
            self.button_map = []

            # Opens the Joystick device
            self.fn = '/dev/input/js0'
            print('    Opening %s:' % self.fn)
            self.js_device = open(self.fn, 'rb')

            # Gets the device name
            buf = array.array('B', [0] * 64)
            ioctl(self.js_device, 0x80006a13 + (0x10000 * len(buf)), buf)
            self.js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
            print('        Device name: %s' % self.js_name)

            # Gets the number of axes and buttons
            buf = array.array('B', [0])
            ioctl(self.js_device, 0x80016a11, buf)
            self.num_axes = buf[0]

            buf = array.array('B', [0])
            ioctl(self.js_device, 0x80016a12, buf)
            self.num_buttons = buf[0]

            # Gets the buttons and axes names from the corresponding Joystick map data
            self.button_names, self.axis_names = joystick_map.joystick_map(self.js_name)

            # Get the axis map.
            buf = array.array('B', [0] * 0x40)
            ioctl(self.js_device, 0x80406a32, buf)
            for axis in buf[:self.num_axes]:
                axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
                self.axis_map.append(axis_name)
                self.axis_states[axis_name] = 0.0

            # Get the button map.
            buf = array.array('H', [0] * 200)
            ioctl(self.js_device, 0x80406a34, buf)
            for btn in buf[:self.num_buttons]:
                btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
                self.button_map.append(btn_name)
                self.button_states[btn_name] = 0.0

            print('        %d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
            print('        %d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

            if self.swarm is not None:
                self.jsl = Thread(target=self.joystick_inputs)
                self.jsl.start()

    def await_packet(self):
        while not self.stopped:
            packet, address = self.network_comm.receiver_socket.recvfrom(4096)
            data = packet.decode('utf-8').split()
            # sender_ip = data[0]
            value = float(data[1])
            command_type = data[2]
            if command_type == 'button':
                button = data[3]
                if button:
                    self.buttons(button, value)
            if command_type == 'axis':
                axis = data[3]
                if axis:
                    self.axis(axis, value)
        self.network_comm.disconnect()

    def joystick_inputs(self):
        while not self.stopped:
            ev_buf = self.js_device.read(8)
            time, value, buf_type, number = struct.unpack('IhBB', ev_buf)

            if buf_type & 0x01:
                button = self.button_map[number]
                if button:
                    self.network_comm.send_packet(str(value) + ' ' + 'button' + ' ' + str(button))
                    self.buttons(button, value)

            if buf_type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    self.network_comm.send_packet(str(value) + ' ' + 'axis' + ' ' + str(axis))
                    self.axis(axis, value)
        self.network_comm.disconnect()

    def buttons(self, button, value):
        if button == 'Stop' and value:
            print('Stop button triggered, joystick disconnected')
            for agt in self.swarm.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            self.stopped = True

        if button == 'Takeoff/Land' and value:
            print('Takeoff / Land button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.is_flying:
                    agt.takeoff()
                elif agt.enabled and agt.is_flying:
                    agt.land()

        if button == 'Standby' and value:
            print('Standby button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying:
                    agt.standby()

        if button == 'Manual_flight' and value:
            print('Manual flight button triggered')
            if any([agt.state == 'z_consensus'
                    or agt.state == 'Takeoff'
                    or agt.state == 'Land' for agt in self.swarm.swarm_agent_list]):
                print('Warning : Manual control command access denied')
                print('          Some UAVs flightmode do not allow them to automatically avoid obstacles.')
                print('          For security reasons, manual flight is prohibited while any UAV is in')
                print('          Takeoff, Land or Height consensus flightmode')
                print('          Please use the Standby, Wingman or Back to initial position flightmode')
                print('          before switching to manual flight to make sure that automatic')
                print('          avoidance is enabled')
            else:
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying and any([agt.name == manual for manual in
                                                              self.swarm.manual_flight_agents_list]):
                        self.swarm.manual_z = agt.extpos.z
                        agt.manual_flight()

        if button == 'Yaw-' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw + 22.5

        if button == 'Yaw+' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw - 22.5

        if button == 'Initial_position' and value:
            print('Initial position button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying:
                    agt.back_to_initial_position()

        if button == 'xy_consensus' and value:
            print('xy consensus button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying:
                    agt.xy_consensus()

        if button == 'z_consensus' and value:
            print('Height consensus button triggered')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying:
                    agt.z_consensus()

        if button == 'Wingman' and value:
            print('Wingman button triggered')
            if self.swarm.swarm_leader is not None:
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying and agt.name == self.swarm.swarm_leader:
                        agt.manual_flight()
                    elif agt.enabled and agt.is_flying and not agt.state == 'Manual':
                        agt.wingman_behaviour()

        if button == 'Pursuit' and value:
            print('Pursuit button triggered')
            if self.swarm.target is not None and self.swarm.chaser is not None:
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying:
                        agt.pursuit()
            else:
                print('Warning : No swarm leader assigned, wingman command access denied')

    def axis(self, axis, value):
        fvalue = value / 32767.0

        if axis == 'pitch':
            if -0.01 < fvalue < 0.01:
                self.swarm.manual_x = 0.0
            else:
                self.swarm.manual_x = fvalue ** 3

        if axis == 'roll':
            if -0.01 < fvalue < 0.01:
                self.swarm.manual_y = 0.0
            else:
                self.swarm.manual_y = fvalue ** 3

        if axis == 'yaw':
            self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * fvalue ** 3

        if axis == 'yaw-':
            fvalue = (1 + fvalue) / 2
            self.swarm.manual_yaw = self.swarm.manual_yaw + 2 * fvalue ** 3

        if axis == 'yaw+':
            fvalue = (1 + fvalue) / 2
            self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * fvalue ** 3

        if axis == 'height':
            self.swarm.manual_z = (1 - fvalue) / 2

        if axis == 'height2':
            self.swarm.manual_z = self.swarm.manual_z - 0.01 * fvalue

    def print_inputs(self):
        """
        Test method that prints any input received from the Joystick
        """
        while True:
            ev_buf = self.js_device.read(8)
            time, value, buf_type, number = struct.unpack('IhBB', ev_buf)

            if buf_type & 0x01:
                button = self.button_map[number]
                if button:
                    print([button, value])

            if buf_type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    print([axis, value])


if __name__ == '__main__':
    js = Joystick()
    js.print_inputs()
