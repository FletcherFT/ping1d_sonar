#!/usr/bin/env python3

"""Scan serial ports for ping devices
    Symlinks to detected devices are created under /dev/serial/ping/
    This script needs root permission to create the symlinks
"""
import subprocess

import numpy as np
import rospy
from brping import PingDevice, PingParser, PingMessage
from brping.definitions import *
import serial
import socket
from collections import deque
from sensor_msgs.msg import Range, MultiEchoLaserScan, LaserEcho


class PingEnumerator:

    def legacy_detect_ping1d(self, ping):
        """
        Detects Ping1D devices without DEVICE_INFORMATION implemented
        """
        firmware_version = ping.request(PING1D_FIRMWARE_VERSION)
        if firmware_version is None:
            return None
        description = "/dev/serial/ping/Ping1D-id-{}-t-{}-m-{}-v-{}.{}".format (
            firmware_version.src_device_id,
            firmware_version.device_type,
            firmware_version.device_model,
            firmware_version.firmware_version_major,
            firmware_version.firmware_version_minor
        )
        return description

    def detect_device(self, dev):
        """
        Attempts to detect the Ping device attached to serial port 'dev'
        Returns the new path with encoded name if detected, or None if the
        device was not detected
        """

        print("Checking if " + dev + " is a Ping device...")
        try:
            ping = PingDevice()
            ping.connect_serial("/dev/serial/by-id/" + dev, 115200)
        except Exception as exception:
            print("An exception has occurred: ", exception)
            return None

        if not ping.initialize():
            return None

        device_info = ping.request(COMMON_DEVICE_INFORMATION)
        if not device_info:
            return self.legacy_detect_ping1d(ping)

        if device_info.device_type == 1:
            description = "/dev/serial/ping/Ping1D-id-{}-r-{}-v-{}.{}.{}"
        elif device_info.device_type == 2:
            description = "/dev/serial/ping/Ping360-id-{}-r-{}-v-{}.{}.{}"
            # Open device with 2M baud to setup Ping360
            print("Setting baud to 2M...")
            ser = serial.Serial("/dev/serial/by-id/" + dev, 2000000)
            ser.send_break()
            ser.write("UUUUUUU".encode())
            ser.close()
            self.set_low_latency(dev)

        else:
            return None

        return description.format (
            device_info.src_device_id,
            device_info.device_revision,
            device_info.firmware_version_major,
            device_info.firmware_version_minor,
            device_info.firmware_version_patch
        )

    def set_low_latency(self, dev):
        """
        Receives /dev/serial/by-id/...
        maps to it to ttyUSB and sets the latency_timer for the device
        """
        raise NotImplementedError("This method currently not supported, requires root permissions.")
        target_device = subprocess.check_output(' '.join(["readlink", "-f", "/dev/serial/by-id/%s" % dev]), shell=True)
        device_name = target_device.decode().strip().split("/")[-1]

        latency_file = "/sys/bus/usb-serial/devices/{0}/latency_timer".format(device_name)

        with open(latency_file, 'w') as p:
            p.write("1")
            p.flush()

    def make_symlink(self, origin, target):
        """
        follows target to real device an links origin to it
        origin => target
        Returns True if sucessful
        """
        raise NotImplementedError("This method currently not supported, requires root permissions.")
        try:
            # Follow link to actual device
            target_device = subprocess.check_output(' '.join(["readlink", "-f", "/dev/serial/by-id/%s" % origin]), shell=True)
            # Strip newline from output
            target_device = target_device.decode().split('\n')[0]

            # Create another link to it
            subprocess.check_output(' '.join(["mkdir", "-p", "/dev/serial/ping"]), shell=True)
            subprocess.check_output("ln -fs %s %s" % (
                target_device,
                target), shell=True)
            print(origin, " linked to ", target)
            return True
        except subprocess.CalledProcessError as exception:
            print(exception)
            return False


    def erase_old_symlinks(self):
        """
        Erases all symlinks at "/dev/serial/ping/"
        """
        raise NotImplementedError("This method currently not supported, requires root permissions.")
        try:
            subprocess.check_output(["rm", "-rf", "/dev/serial/ping"])
        except subprocess.CalledProcessError as exception:
            print(exception)


    def list_serial_devices(self):
        """
        Lists serial devices at "/dev/serial/by-id/"
        """
        # Look for connected serial devices
        try:
            output = subprocess.check_output("ls /dev/serial/by-id", shell=True)
            return output.decode().strip().split("\n")
        except subprocess.CalledProcessError as exception:
            print(exception)
            return []


class PingDriver:
    def __init__(self):
        rospy.init_node("ping1d_driver_node")
        self.ping_sensors = []
        self.enumerator = PingEnumerator()
        hz = rospy.Rate(1.0)
        while not len(self.ping_sensors) and not rospy.is_shutdown():
            self.ping_sensors = [f"/dev/serial/by-id/{dev}" for dev in self.enumerator.list_serial_devices()]
            rospy.logerr_throttle(10.0, f"{rospy.get_name()} | Waiting for valid ping1d sensor to appear.")
            hz.sleep()

        ## Messages that have the current distance measurement in the payload
        self.distance_messages = [
            PING1D_DISTANCE,
            PING1D_DISTANCE_SIMPLE,
            PING1D_PROFILE
        ]

        ## Parser to verify client comms
        self.parser = PingParser()

        self.range_publisher = rospy.Publisher("range", Range, queue_size=10)
        self.profile_publisher = rospy.Publisher("profile", MultiEchoLaserScan, queue_size=10)
        self.hz = rospy.Rate(15.0)

        if not rospy.is_shutdown():
            rospy.loginfo("Setting up serial device.")
            self.device = PingDevice()
            self.device.connect_serial(self.ping_sensors[0], 115200)
            data = PingMessage(PING1D_CONTINUOUS_STOP)
            data.pack_msg_data()
            self.device.write(data.msg_data)
            data = PingMessage(PING1D_SET_MODE_AUTO)
            data.pack_msg_data()
            self.device.write(data.msg_data)
            data = PingMessage(PING1D_SET_RANGE)
            data.scan_start = 200
            data.scan_length = 30000
            data.pack_msg_data()
            self.device.write(data.msg_data)
    ## Digest incoming ping data
    def parse(self, data: PingMessage):
        range_msg = None
        profile_msg = None
        if data.message_id in self.distance_messages:
            range_msg = Range()
            range_msg.header.frame_id = "altimeter"
            range_msg.header.stamp = rospy.Time.now()
            range_msg.radiation_type = range_msg.ULTRASOUND
            range_msg.field_of_view = 0.52
            range_msg.max_range = (data.scan_start + data.scan_length) / 1000
            range_msg.min_range = data.scan_start / 1000.0
            if range_msg.min_range <= data.distance / 1000 <= range_msg.max_range:
                range_msg.range = data.distance / 1000
        if data.message_id == PING1D_PROFILE:
            profile_msg = MultiEchoLaserScan()
            profile_msg.header = range_msg.header
            profile_msg.ranges = [LaserEcho(np.linspace(data.scan_start / 1000, data.scan_start / 1000 + data.scan_length / 1000, data.profile_data_length).tolist())]
            profile_msg.range_min = data.scan_start / 1000.0
            profile_msg.range_max = (data.scan_start + data.scan_length) / 1000
            profile_msg.angle_increment = 0
            profile_msg.angle_max = 0
            profile_msg.angle_min = 0
            profile_msg.intensities = [LaserEcho(np.frombuffer(data.profile_data, dtype=np.uint8).tolist())]
        return range_msg, profile_msg

    def send_ping1d_request(self):
        data = PingMessage()
        data.request_id = PING1D_DISTANCE
        data.src_device_id = 0
        data.pack_msg_data()
        self.device.write(data.msg_data)

    def run(self):
        # read ping device from serial
        try:
            while not rospy.is_shutdown():
                self.send_ping1d_request()
                device_data = self.device.read()
                if device_data is not None:
                    range_msg, profile_msg = self.parse(device_data)
                    if range_msg is not None:
                        self.range_publisher.publish(range_msg)
                    if profile_msg is not None:
                        self.profile_publisher.publish(profile_msg)
                self.hz.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.device.iodev.close()


class PingClient(object):
    def __init__(self):
        ## Queued messages received from client
        self.rx_msgs = deque([])

        ## Parser to verify client comms
        self.parser = PingParser()

    ## Digest incoming client data
    # @return None
    def parse(self, data):
        for b in bytearray(data):
            if self.parser.parse_byte(b) == PingParser.NEW_MESSAGE:
                self.rx_msgs.append(self.parser.rx_msg)

    ## Dequeue a message received from client
    # @return None: if there are no comms in the queue
    # @return PingMessage: the next ping message in the queue
    def dequeue(self):
        if len(self.rx_msgs) == 0:
            return None
        return self.rx_msgs.popleft()


class PingProxy(object):
    def __init__(self, device: str, port: int, topic: str):
        ## A serial object for ping device comms
        self.device = device

        ## UDP port number for server
        self.port = port

        ## Publisher to send ROS range information on
        self.range_msg = Range()
        self.range_publisher = rospy.Publisher(topic, Range, queue_size=10)

        ## Connected client dictionary
        self.clients = {}

        ## Socket to serve on
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setblocking(False)
        self.socket.bind(('0.0.0.0', self.port))

    ## Run proxy tasks
    def run(self):
        try:
            data, address = self.socket.recvfrom(4096)

            # new client
            if address not in self.clients:
                self.clients[address] = PingClient()

            # digest data coming in from client
            self.clients[address].parse(data)
        except TimeoutError:
                pass  # waiting for data
        except Exception as e:
            print("Error reading data", e)

        # read ping device from serial
        device_data = self.device.read(self.device.in_waiting)

        # send ping device data to all clients via UDP
        if device_data:  # don't write empty data
            for client in self.clients:
                # print("writing to client", client)
                self.socket.sendto(device_data, client)

        # send all client comms to ping device
        for client in self.clients:
            c = self.clients[client]
            msg = c.dequeue()
            while msg is not None:
                self.device.write(msg.msg_data)
                msg = c.dequeue()
