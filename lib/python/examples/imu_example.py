#!/usr/bin/python3

# Copyright 2022 Josh Pieper, jjp@pobox.com.
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

'''Demonstrates using the IMU API of the pi3hat.'''

import asyncio
import moteus
import moteus_pi3hat


async def main():
    transport = moteus_pi3hat.Pi3HatRouter()

    while True:
        # When request_attitude=True, the attitude is returned as a
        # response from a "special" servo with ID -1 and the type
        # moteus_pi3hat.CanAttitudeWrapper.
        result = await transport.cycle([], request_attitude=True)
        imu_result = [
            x for x in result
            if x.id == -1 and isinstance(x, moteus_pi3hat.CanAttitudeWrapper)][0]

        # It has fields for the attitude quaternion, the angular rate
        # in dps, and the acceleration in mps2.  Additionally, for
        # convenience, the quaternion is converted into euler angles
        # following the quad A1 convention.

        att = imu_result.attitude
        print(f"attitude={att.w:.4f},{att.x:.4f},{att.y:.4f},{att.z:.4f}")

        rate_dps = imu_result.rate_dps
        print(f"rate_dps={rate_dps.x:.3f},{rate_dps.y:.3f},{rate_dps.z:.3f}")

        accel_mps2 = imu_result.accel_mps2
        print(f"accel_mps2={accel_mps2.x:.3f},{accel_mps2.y:.3f},{accel_mps2.z:.3f}")

        euler_rad = imu_result.euler_rad
        print(f"euler_rad= r={euler_rad.roll:.3f},p={euler_rad.pitch:.3f},y={euler_rad.yaw:.3f}")

        print()

        await asyncio.sleep(0.5)


if __name__ == '__main__':
    asyncio.run(main())
