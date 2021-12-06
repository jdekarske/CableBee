import beecontrol

JASON_HOME = [1445, 1143, 280]
ADAPTER_HOME = [1500,1100,2200]

mybee = beecontrol.Bee(*JASON_HOME)
print(mybee.currentStringLengths())
mybee.setStringPosition()
# mybee.setHome(*JASON_HOME)

# desk high room square
mybee.steppers.sendCommand('G0 F3000')
mybee.absoluteMove(600,600,1200)
mybee.absoluteMove(2400,600,1200)
mybee.absoluteMove(600,2000,1200)
mybee.absoluteMove(2400,2000,1200)

# do a square #20mm/s/s is conservative acceleration, 10 is slow, 40 good
mybee.steppers.sendCommand('G0 F2000')
mybee.absoluteMove(1500,1000,1500)
mybee.absoluteMove(1500,1500,1500)
mybee.absoluteMove(1500,1500,1000)
mybee.absoluteMove(1500,1000,1000)

# go in a circle
import numpy as np
r = 500
k = 1000 
h = 1250
xs = np.arange(h-r,h+r,10)
zs = np.sqrt(r**2 - (xs-h)**2) + k

import time

mybee.steppers.sendCommand('G0 F4000')
for x,z in zip(xs,zs):
    mybee.absoluteMove(x,1000,z)

print("hi")

