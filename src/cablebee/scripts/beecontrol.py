import serialcontrol as sc
import numpy as np
import time


class Bee:
    DEFAULT_CURRENT = 1000

    # room dimensions centered in door corner
    roomX = 3100  # along North wall
    roomY = 3011  # along West wall
    roomZ = 2450

    # anchor positions for wheel
    anchorX = np.array([roomX-120,   roomY-93,   roomZ-30])
    anchorY = np.array([155,         roomY-85,   roomZ-30])
    anchorZ = np.array([roomX-103,   112,        roomZ-30])
    anchorE = np.array([130,         95,         roomZ-30])

    anchors = [anchorX, anchorY, anchorZ, anchorE]

    # offsets (faces away from bee)
    # offset = 80  # mm
    # offsetX = np.array([offset/np.sqrt(2),  offset/np.sqrt(2), 0])
    # offsetY = np.array([-offset/np.sqrt(2), offset/np.sqrt(2), 0])
    # offsetZ = np.array([offset/np.sqrt(2),  -offset/np.sqrt(2), 0])
    # offsetE = np.array([-offset/np.sqrt(2), -offset/np.sqrt(2), 0])

    # offset positions for point
    offsetX = np.array([0,   0,   0])
    offsetY = np.array([0,   0,   0])
    offsetZ = np.array([0,   0,   0])
    offsetE = np.array([0,   0,   0])

    offsets = [offsetX, offsetY, offsetZ, offsetE]

    steppers = None

    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z
        self.currentPosition = [self.x, self.y, self.z]
        self.steppers = sc.StepperController()
        self.steppers.enableSteppers()

    def getStringLengths(self, x, y, z):
        stringlengths = []
        C = [x, y, z]
        for (anchor, offset) in zip(self.anchors, self.offsets):
            string_vec = C + offset - anchor
            stringlength = np.linalg.norm(string_vec)
            stringlengths.append(np.floor(stringlength))
        return np.array(stringlengths).astype(int)

    def currentStringLengths(self):
        return self.getStringLengths(self.x, self.y, self.z)

    def setStringPosition(self):
        self.steppers.setPosition(*self.currentStringLengths())

    def absoluteMove(self, x, y, z):
        self.steppers.absolutePositioning()
        self.steppers.linearMove(*self.getStringLengths(x, y, z))

    # This method will tension and release the strings repeatedly to fully tension cablebee
    #   (needs some work)
    # def setHome(self, x, y, z):
    #     self.steppers.setCurrent(1, 1, 1, 1) # make the stall current minimal
    #     self.steppers.relativePositioning()
    #     for i in range(10):
    #         self.steppers.enableSteppers()
    #         self.steppers.linearMove(-2, -2, -2, -2)
    #         self.steppers.disableSteppers()
    #         time.sleep(0.5)
    #     self.steppers.enableSteppers()
    #     self.steppers.setCurrent(
    #     self.DEFAULT_CURRENT, self.DEFAULT_CURRENT, self.DEFAULT_CURRENT, self.DEFAULT_CURRENT)
    #     self.steppers.linearMove(-15, -15, -15, -15) # tension the strings
    #     [self.x, self.y, self.z] = x, y, z
    #     self.setStringPosition()
