#!/usr/bin/python3

from helpers import getch

import rospy
from std_msgs.msg import Float64

class ROSCom:

    def __init__(self):
        self._pub = None
        rospy.init_node('interactive_ctrl')

    @property
    def motorid(self):
        return self._motorid

    @motorid.setter
    def motorid(self, val):
        self._motorid = val
        if self._pub:
            self._pub.unregister()
        self._pub = rospy.Publisher("%s/command" % val, Float64, queue_size=1)

    def send(self, angle):
        self._pub.publish(angle)

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
        self._askmotor()

        while True:
            self._printcursor()
            self.com.send(self.val)
            
            key = self._getkey()

            if key.lower() == 'q':
                break
            elif key == self.UP:
                self._increase()
            elif key == self.DOWN:
                self._decrease()
            elif key == self.RIGHT:
                self._lesser_digit()
            elif key == self.LEFT:
                self._greater_digit()

            self._printcursor()

    def _askmotor(self):
        self.com.motorid = input('Enter Motor ID: ')
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
