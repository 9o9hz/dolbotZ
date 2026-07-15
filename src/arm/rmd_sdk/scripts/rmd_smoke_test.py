#!/usr/bin/env python3
"""
Read-only smoke test for a MyActuator RMD X-series motor over SocketCAN.

Default behavior is intentionally safe: it only opens the CAN interface and
queries firmware/status values. Motion requires both --move-position and
--yes-move.
"""

import argparse
import sys
import time


def import_rmd():
    try:
        import rmd_sdk_py as rmd

        return rmd
    except ImportError as direct_error:
        try:
            import rmd_sdk.rmd_sdk_py as rmd

            return rmd
        except ImportError:
            print("Could not import rmd_sdk_py.")
            print("Install first with: pip3 install .")
            print(f"Original import error: {direct_error}")
            sys.exit(2)


def call(name, fn):
    try:
        value = fn()
        print(f"{name}: {value}")
        return value
    except Exception as exc:  # Hardware/protocol errors come from pybind types.
        print(f"{name}: FAILED ({type(exc).__name__}: {exc})")
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Safe first-contact test for RMD X-series motors."
    )
    parser.add_argument("-i", "--interface", default="can_arm", help="SocketCAN interface")
    parser.add_argument(
        "--id",
        type=int,
        default=1,
        help="Actuator ID configured in the motor, usually 1..32",
    )
    parser.add_argument(
        "--move-position",
        type=float,
        help="Optional absolute position command in degrees",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=30.0,
        help="Max speed for --move-position in deg/s",
    )
    parser.add_argument(
        "--yes-move",
        action="store_true",
        help="Required together with --move-position to allow motion",
    )
    parser.add_argument(
        "--shutdown",
        action="store_true",
        help="Call shutdownMotor() before exit",
    )
    args = parser.parse_args()

    if args.id < 1 or args.id > 32:
        parser.error("--id must be in the range 1..32")

    if args.move_position is not None and not args.yes_move:
        parser.error("motion requires --yes-move")

    rmd = import_rmd()
    print(f"Opening {args.interface}, actuator ID {args.id}")
    print(f"Expected CAN address: 0x{0x140 + args.id:03x}")

    driver = rmd.CanDriver(args.interface)
    actuator = rmd.ActuatorInterface(driver, args.id)

    call("firmware version date", actuator.getVersionDate)
    call("motor model", actuator.getMotorModel)
    call("control mode", actuator.getControlMode)
    call("motor status 1", actuator.getMotorStatus1)
    call("motor status 2", actuator.getMotorStatus2)
    call("multi-turn angle", actuator.getMultiTurnAngle)

    if args.move_position is not None:
        print(
            "Sending absolute position command: "
            f"{args.move_position} deg at max {args.max_speed} deg/s"
        )
        call(
            "move feedback",
            lambda: actuator.sendPositionAbsoluteSetpoint(
                args.move_position, args.max_speed
            ),
        )
        time.sleep(0.5)
        call("motor status 2 after move", actuator.getMotorStatus2)

    if args.shutdown:
        print("Shutting down motor")
        actuator.shutdownMotor()


if __name__ == "__main__":
    main()
