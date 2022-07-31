#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from ping360_sonar.cfg import sonarConfig
from ping360_sonar.msg import SonarEcho
from sonar_interface import SonarInterface, Sector


class Ping360_Node():
    def __init__(self):
        # init node
        rospy.init_node('ping360_node')

        # topic publishers
        self.publish_image = rospy.get_param('~enableImageTopic', True)
        self.publish_scan = rospy.get_param('~enableScanTopic', True)
        self.publish_echo = rospy.get_param('~enableDataTopic', True)
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                "/ping360_node/sonar/images", Image, queue_size=1)
            rospy.Timer(rospy.Duration(self.image_rate / 1000), self.publishImage)
        if self.publish_scan:
            self.scan_pub = rospy.Publisher("/ping360_node/sonar/data",
                                            SonarEcho, queue_size=1)
        if self.publish_echo:
            self.echo_pub = rospy.Publisher(
                "/ping360_node/sonar/scan", LaserScan, queue_size=1)

        # init sonar interface
        device = rospy.get_param('~device', "/dev/ttyUSB0")
        baudrate = rospy.get_param('~baudrate', 115200)
        fallback_emulator = True
        connection_type = 'serial'
        self.sonar = SonarInterface(device, baudrate, fallback_emulator, connection_type)

        # sonar config
        self.gain = rospy.get_param('~gain', 0)                         # range: 0 - 2
        self.frequency = rospy.get_param('~transmitFrequency', 740)
        self.scan_threshold = int(rospy.get_param('~threshold', 200))   # range: 0 - 255
        self.angle_step = int(rospy.get_param('~step', 1))              # range: 1 - 20
        self.speed_of_sound = rospy.get_param('~speedOfSound', 1500)    # range: 1350 - 1750
        self.range_max = rospy.get_param('~sonarRange', 2)              # range: 1 - 50
        self.angle_sector = 360     # range: 60 - 360
        self.image_size = 500       # range: 200 - 1000
        self.image_rate = 100       # range: 50 - 2000
        self.sonar.configureAngles(self.angle_sector,
                                   self.angle_step,
                                   self.publish_scan)
        self.sonar.configureTransducer(self.gain,
                                       self.frequency,
                                       self.speed_of_sound,
                                       self.range_max)

        # init messages
        frame = "sonar"
        # image msg
        self.image = Image()
        self.image.header.frame_id = frame
        self.image.encoding = 'mono8'
        self.image.is_bigendian = 0
        self.image.step = self.image.width = self.image.height = self.image_size
        self.image.data = [0 for _ in range(self.image_size * self.image_size)]
        # laser scan msg
        self.scan = LaserScan()
        self.scan.header.frame_id = frame
        self.scan.range_min = 0.75
        self.scan.range_max = float(self.range_max)
        self.scan.time_increment = self.sonar.transmitDuration()
        self.scan.angle_min = self.sonar.angleMin()
        self.scan.angle_max = self.sonar.angleMax()
        self.scan.angle_increment = self.sonar.angleStep()
        # sonar echo msg
        self.echo = SonarEcho()
        self.echo.header.frame_id = frame
        self.echo.gain = self.gain
        self.echo.range = self.range_max
        self.echo.speed_of_sound = self.range_max
        self.echo.number_of_samples = self.sonar.samples
        self.echo.transmit_frequency = self.frequency

        # sonar sector
        self.sector = Sector()
        self.sector.configure(self.sonar.samples, self.image_size // 2)

    def refresh(self):

        valid, end_turn = self.sonar.read()

        if not valid:
            return

        if self.publish_echo:
            self.publishEcho()

        if self.publish_image:
            self.refreshImage()

        if self.publish_scan:
            self.publishScan(end_turn)

    def publishEcho(self):
        """
        Publishes the last raw echo message
        """
        self.echo.angle = self.sonar.currentAngle()
        self.echo.intensities = self.sonar.data
        self.echo.header.stamp = rospy.Time.now()
        self.echo_pub.publish(self.echo)

    def publishScan(self, end_turn):
        """
        Updates the laserScan message for the scan topic
        Actually publishes only after the end of each turn
        """
        count = self.sonar.angleCount()
        cur = len(self.scan.ranges)
        for _ in range(count - cur):
            self.scan.ranges.append(0.)
            self.scan.intensities.append(0.)
        if cur > count:
            self.scan.ranges = self.scan.ranges[:count]
            self.scan.intensities = self.scan.intensities[:count]

        cur = self.sonar.angleIndex()
        for i in range(len(self.sonar.data)):
            if self.sonar.data[i] >= self.scan_threshold:
                dist = self.sonar.rangeFrom(i)
                if self.scan.range_min <= dist <= self.scan.range_max:
                    self.scan.ranges = dist
                    self.scan.intensities = self.sonar.data[i] / 255.
                    break

        if end_turn and not self.sonar.fullScan():
            if self.sonar.angleStep() < 0:
                # now going negative: scan was positive
                self.scan.angle_max = self.sonar.angleMax()
                self.scan.angle_min = self.sonar.angleMin()
            else:
                # now going positive: scan was negative
                self.scan.angle_max = self.sonar.angleMin()
                self.scan.angle_min = self.sonar.angleMax()
            self.scan.angle_increment = -self.sonar.angleStep()
            self.scan.angle_max -= self.scan.angle_increment

            self.scan.header.stamp = rospy.Time.now()
            self.scan_pub.publish(self.scan)

    def publishImage(self):
        self.image.header.stamp = rospy.Time.now()
        self.image_pub.publish(self.image)

    def refreshImage(self):
        half_size = self.image.step // 2
        self.sector.init(self.sonar.currentAngle(), self.sonar.angleStep())
        length = len(self.sonar.data)
        x = 0
        y = 0
        more_points, x, y, index = self.sector.nextPoint(x, y)
        while more_points:
            if index < length:
                self.image.data[half_size - y + self.image.step * (half_size - x)] = self.sonar.data[index]
            more_points, x, y, index = self.sector.nextPoint(x, y)
