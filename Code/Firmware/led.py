from pycom import rgbled, heartbeat
from machine import Timer
import utime


class LED(object):
    RED = 0xff0000
    GREEN = 0x00FF00
    YELLOW = 0x7f7f00
    BLUE = 0x0000FF
    blinking = False
    _state = False
    _blink = None
    _color = RED

    def rgb_to_hex(self, red, green, blue):
        """Return color as #rrggbb for the given color values."""
        return '%02x%02x%02x' % (red, green, blue)

    def cycle(self, color, count, leave_color=False):
        hbon = heartbeat()  # stash for revert later
        if hbon:
            heartbeat(False)
        for j in range(count):
            self.on(color)
            if color == self.RED:
                for i in range(256):
                    col = self.rgb_to_hex(i, 0, 0)
                    rgbled(256 - int(col, 16))
                    utime.sleep_ms(1)
            if color == self.GREEN:
                for i in range(256):
                    col = self.rgb_to_hex(0, i, 0)
                    rgbled(256 - int(col, 16))
                    utime.sleep_ms(1)
            if color == self.BLUE:
                for i in range(256):
                    col = self.rgb_to_hex(0, 0, i)
                    rgbled(256 - int(col, 16))
                    utime.sleep_ms(1)
        if leave_color:
            self.on(color)
        heartbeat(hbon)

    def on(self, color=None):
        if color:
            self._color = color
        rgbled(self._color)

    def heartbeat_on(self):
        heartbeat(True)

    def off(self):
        heartbeat(False)

    def stop_blink(self):
        self._blink = None
        self.off()

    def start_blink(self):
        pass

    def blink_red(self, alarm):
        if self._state:
            self.red()
        else:
            self.off()
        self._state = not self._state
        if not self.blinking:
            alarm.cancel()
        else:
            self._blink = Timer.Alarm(self.blink_red, 1)


if __name__ == "__main__":
    import utime

    led = LED()
    led.off()
    led.on(led.RED)
    utime.sleep_ms(250)
    led.on(led.YELLOW)
    utime.sleep_ms(250)
    led.on(led.GREEN)
    utime.sleep_ms(250)
    led.on(led.BLUE)
    utime.sleep_ms(250)
    led.cycle(led.RED, 5)
    led.cycle(led.GREEN, 5)
    led.cycle(led.BLUE, 5)

    led.off()

    led.heartbeat_on()
