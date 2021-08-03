import beecontrol
from pynput.keyboard import Key, Listener

class KeyHandler:
    MOVE_DISTANCE = 20
    move_multiplier = 1

    def on_press(self, key):
        try:
            thiskey = key.char
        except AttributeError:
            #some keys don't have char
            return

        mybee.steppers.relativePositioning()

        # init dist changes
        x,y,z,a = (0,0,0,0)

        # extend the line
        if thiskey == 'a':
            x += self.MOVE_DISTANCE
        if thiskey == 's':
            y += self.MOVE_DISTANCE
        if thiskey == 'd':
            z += self.MOVE_DISTANCE
        if thiskey == 'f':
            a += self.MOVE_DISTANCE
            
        # retract the line
        if thiskey == 'z':
            x -= self.MOVE_DISTANCE
        if thiskey == 'x':
            y -= self.MOVE_DISTANCE
        if thiskey == 'c':
            z -= self.MOVE_DISTANCE
        if thiskey == 'v':
            a -= self.MOVE_DISTANCE

        # extend all
        if thiskey == 'g':
            x += self.MOVE_DISTANCE
            y += self.MOVE_DISTANCE
            z += self.MOVE_DISTANCE
            a += self.MOVE_DISTANCE

        # retract all
        if thiskey == 'b':
            x -= self.MOVE_DISTANCE
            y -= self.MOVE_DISTANCE
            z -= self.MOVE_DISTANCE
            a -= self.MOVE_DISTANCE

        # small move
        if thiskey == 'q':
            self.move_multiplier = 1
        
        #big move
        if thiskey == 'w':
            self.move_multiplier = 4

        # do the move
        mybee.steppers.linearMove(x*self.move_multiplier,y*self.move_multiplier,z*self.move_multiplier,a*self.move_multiplier)


    def on_release(self, key):
        # print('{0} release'.format(
        #     key))
        if key == Key.esc:
            # Stop listener
            return False

if __name__ == "__main__":
    mybee = beecontrol.Bee()
    mybee.steppers.setPosition(0, 0, 0, 0)
    
    kh = KeyHandler()

    # Collect events until released
    with Listener(
            on_press=kh.on_press,
            on_release=kh.on_release) as listener:
        listener.join()




# do a square #20mm/s/s is conservative acceleration, 10 is slow, 40 good
# mybee.steppers.sendCommand('G0 F6000')
# mybee.absoluteMove(1500,1000,1500)
# mybee.absoluteMove(1500,1500,1500)
# mybee.absoluteMove(1500,1500,1000)
# mybee.absoluteMove(1500,1000,1000)
