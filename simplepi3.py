import asyncio
import math
import moteus
import moteus_pi3hat
import time


async def main():

    f = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={...},
    )
    c = moteus.Controller(transport=f, id=1)

    await c.set_stop()

    pos = 0.5

    for i in range(4):
        value = 0 - pos

        pos = value

        state = await c.set_position(position=math.nan, velocity=0.2, maximum_torque=0.3, 
                        stop_position=pos, watchdog_timeout=10, query=True)

        await asyncio.sleep(6)

        print(state)

        print("Position:", state.values[moteus.Register.POSITION])

        print()

    await asyncio.sleep(3)

    await c.set_stop()

if __name__ == '__main__':
    asyncio.run(main())
