#!/usr/bin/python3 -B

# Copyright 2021 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This example commands a single servo at ID #1 using the default
transport to hold the current position indefinitely, and prints the
state of the servo to the console.
"""

import asyncio
import math
import moteus
import time


async def main():
    # By default, Controller connects to id 1, and picks an arbitrary
    # CAN-FD transport, prefering an attached fdcanusb if available.
    #f = moteus.Fdcanusb('COM5')
    #c = moteus.Controller(transport=f, id=1)
    c = moteus.Controller()

    # In case the controller had faulted previously, at the start of
    # this script we send the stop command in order to clear it.
    await c.set_stop()

    pos = 0.5

    for i in range(4):
        value = 0 - pos

        pos = value
        # `set_position` accepts an optional keyword argument for each
        # possible position mode register as described in the moteus
        # reference manual.  If a given register is omitted, then that
        # register is omitted from the command itself, with semantics
        # as described in the reference manual.
        #
        # The return type of 'set_position' is a moteus.Result type.
        # It has a __repr__ method, and has a 'values' field which can
        # be used to examine individual result registers.
        state = await c.set_position(position=math.nan, velocity=0.2, maximum_torque=0.3, stop_position=pos, watchdog_timeout=10, query=True)

        await asyncio.sleep(6)

        # Print out everything.
        print(state)

        # Print out just the position register.
        print("Position:", state.values[moteus.Register.POSITION])

        # And a blank line so we can separate one iteration from the
        # next.
        print()

        # Wait 20ms between iterations.  By default, when commanded
        # over CAN, there is a watchdog which requires commands to be
        # sent at least every 100ms or the controller will enter a
        # latched fault state.

    await asyncio.sleep(3)

    await c.set_stop()

if __name__ == '__main__':
    asyncio.run(main())
