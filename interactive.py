#!/usr/bin/python3

from helpers import getch

import rospy
from std_msgs.msg import Float64
from ros_pololu_servo.msg import servo_pololu

class ROSCom:

    class Hardware:

        _pub = None

        def set_topic(self, val):
            self.drop()
            self._topic = val

        def drop(self):
            if self._pub:
                self._pub.unregister()
                self._pub = None

        # def setmotorid(self, val)
        set_motorid = NotImplemented

        def send(self, angle):
            raise NotImplementedError

    class Pololu(Hardware):

        SPEED = 50
        ACCELERATION = 10

        def set_motorid(self, val):
            self._motorid = int(val)

        def send(self, angle):
            if not self._pub:
                self._pub = rospy.Publisher(self._topic, servo_pololu,
                                            queue_size=10)
            self._pub.publish(servo_pololu(self._motorid, angle,
                                           self.SPEED, self.ACCELERATION))

    class Dynamixel(Hardware):

        def send(self, angle):
            if not self._pub:
                self._pub = rospy.Publisher(self._topic, Float64, queue_size=10)

            self._pub.publish(angle)

    def __init__(self):
        self._pub = None
        self.hardware = None
        rospy.init_node('interactive_ctrl')

    def set_hardware(self, name):
        if self.hardware:
            self.hardware.drop()
            self.hardware = None

        name = name.lower()
        if 'pol' in name:
            self.hardware = self.Pololu()
            return 'Pololu'
        elif 'dyn' in name or 'dxl' in name:
            self.hardware = self.Dynamixel()
            return 'Dynamixel'
        return None


class UserInterface:

    UP = '\x1b[A'
    DOWN = '\x1b[B'
    RIGHT = '\x1b[C'
    LEFT = '\x1b[D'

    def __init__(self, com):
        self.com = com # network communication interface

        self.motorid = None
        self.val = None # current angle/value
        self.seldigit = 0 # selected digit

    def spin(self):
        self._askall()

        while True:
            self.com.hardware.send(self.val)
            self._printcursor()

            key = self._getkey()

            if key.lower() == 'q':
                break
            elif key.lower() == 'h':
                self._askall()
            elif key.lower() == 't':
                self._asktopic()
                self._askangle()
            elif key.lower() == 'm':
                self._askmotor()
                self._askangle()
            elif key == self.UP:
                self._increase()
            elif key == self.DOWN:
                self._decrease()
            elif key == self.RIGHT:
                self._lesser_digit()
            elif key == self.LEFT:
                self._greater_digit()

    def _askall(self):
        self._askhardware()
        self._asktopic()
        self._askmotor()
        self._askangle()

    def _askhardware(self):
        recognizedname = None
        while not recognizedname:
            recognizedname = self.com.set_hardware(input('Dynamixel or Pololu? '))
        print('Using %s' % recognizedname)

    def _asktopic(self):
        self.com.hardware.set_topic(input('Enter publisher topic: '))

    def _askmotor(self):
        if self.com.hardware.set_motorid == NotImplemented:
            print('No motor id required')
        else:
            self.com.hardware.set_motorid(input('Enter motor id: '))

    def _askangle(self):
        self.val = float(input('Enter Initial Angle: '))


    def _printcursor(self):

        def _doublesplit(string, index, prefix=None, trail=None):
            '''split string into preindex, at index and postindex parts

            @param integer string
                if not in range(len(string)), string will be extended with
                prefix or trail
            '''
            #extend the string with prefix and trail according to index position
            if index < 0 and prefix:
                string = prefix*(-index) + string
                index = 0

            if index+1 > len(string) and trail:
                string = string + trail*(index+1-len(string))

            #split
            start, middle, end = string[:index], string[index], string[index+1:]
            return start, middle, end

        # wrap underscores around currently selected digit
        strval = '%f' % self.val
        index_point = strval.find('.')
        if self.seldigit >= 0:
            strval = '_'.join(_doublesplit(strval,
                                           index_point - self.seldigit - 1,
                                           prefix='0'))
        else:
            strval = '_'.join(_doublesplit(strval,
                                           index_point - self.seldigit,
                                           trail='0'))

        print('Angle: %s' % strval)


    def _increase(self):
        self.val += 10**self.seldigit

    def _decrease(self):
        self.val -= 10**self.seldigit

    def _lesser_digit(self):
        self.seldigit -= 1

    def _greater_digit(self):
        self.seldigit += 1


    @staticmethod
    def _getkey():
        result = getch()
        if result == '\x1b':
            result += getch() + getch()
        return result



if __name__ == '__main__':
    com = ROSCom()
    UserInterface(com).spin()
