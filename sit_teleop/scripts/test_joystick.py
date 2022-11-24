from pyjoystick.sdl2 import Key, Joystick, run_event_loop


def print_add(joy: Joystick):
    print(f'{joy.name} Connected', )


def print_remove(joy: Joystick):
    print(f'{joy.name} Disconnected', )


def key_received(key: Key):
    print(f'{key.keyname}: {key.value}')


run_event_loop(print_add, print_remove, key_received)
