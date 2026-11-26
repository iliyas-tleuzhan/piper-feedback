#!/usr/bin/env python3
"""Independent receive-only JSONL trace of CAN arrival gaps and enable traffic."""
import argparse
import json
import select
import signal
import struct
import time
from contextlib import ExitStack
from pathlib import Path

from piper_x_haptics import bus, receive_stamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with ExitStack() as stack:
        log = stack.enter_context(args.output.open('a', buffering=1))
        channels = ('can_master', 'can_slave')
        sockets = [stack.enter_context(bus(name)) for name in channels]
        def write(event, **fields):
            log.write(json.dumps(dict(time=time.time(), event=event, **fields))+'\n')
        write('start')
        last = [{}, {}]
        counts = [0, 0]
        report_at = time.monotonic()
        while running:
            for sock in select.select(sockets, [], [], .02)[0]:
                index = sockets.index(sock)
                try:
                    frame, arrival = receive_stamped(sock)
                except BlockingIOError:
                    continue
                identifier, size, data = struct.unpack('=IB3x8s', frame)
                counts[index] += 1
                last[index][identifier] = arrival
                if identifier in (0x470, 0x471, 0x151):
                    write('command', can=channels[index], identifier=hex(identifier),
                          data=data[:size].hex(), arrival=arrival)
            now = time.monotonic()
            if now >= report_at:
                for index in range(2):
                    write('rx', can=channels[index], count=counts[index],
                          ages_ms={hex(i): round((now-t)*1000, 2)
                                   for i, t in last[index].items()
                                   if i in (0x2a1, 0x2a5, 0x251, 0x261, 0x266, 0x155)})
                report_at = now+.1
        write('stop')


if __name__ == '__main__':
    main()
