from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Color
import gc

gc.enable()
gc.collect()
hub = PrimeHub()

hub.imu.reset_heading(0)
motorL = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
motorR = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
motorL.control.limits(acceleration=2500)
motorR.control.limits(acceleration=2500)

rightEngine = Motor(Port.E)
leftEngine = Motor(Port.A)
robot = DriveBase(motorL, motorR, 56, 160)

baseSpeed = 500
baseAngle = 0

print("----------------------------------------------------------")
print("--------------------- Start of code ----------------------")
print("----------------------------------------------------------")

def move_cm(distance, speed = baseSpeed, ramp_up=0.1, ramp_down=1.9):
    # Reset angles
    # Wheel diameter 5.6cm -> circumference = 5.6 * 3.14159 = 17.593cm
    wheel_circ = 17.593
    distance = (distance / wheel_circ) * 360

    motorL.reset_angle(0)
    motorR.reset_angle(0)

    ramp_up = (speed / 500) * ramp_up * 1.5
    ramp_down = (speed / 500) * ramp_down * 1.5

    ramp_up = (ramp_up / wheel_circ) * 360
    ramp_down = (ramp_down / wheel_circ) * 360

    track_dist = 22
    kp = 5.5
    i = 0

    # Calculate if we use the built in acceleration profile or to split the drive into 2
    if ramp_up + ramp_down > abs(distance):
        ramp_up = abs(distance) / 2
        ramp_down = ramp_up

    if distance < 0:
        direction = -1
    else:
        direction = 1

    # ----------------------------------------------------------------------------------------------------------------------------------
    gc.collect()
    gc.disable()
    avgAngle = (motorR.angle() + motorL.angle()) / 2
    while abs((motorR.angle() + motorL.angle()) / 2) < abs(distance) - track_dist:
        # Get the direction
        heading = hub.imu.heading()

        # Get the average angle of both motors
        avgAngle = (motorR.angle() + motorL.angle()) / 2

        # Calculate the percentage of the track left in the braking and acceleration zones
        percent_to = abs(avgAngle / ramp_up)
        percent_from = abs((((abs(avgAngle)) - (abs(distance) - ramp_down - track_dist)) / (ramp_down)) - 1)

        # Decide if we are ramping up, slowing down or staying at constant velocity
        if abs(avgAngle) < ramp_up and percent_to < 1:
            percent = min(1.0, percent_to + 0.2)
            hub.light.on(Color.VIOLET)
            #print("UP")
        elif abs(avgAngle) > abs(distance) - ramp_down and percent_from < 1:
            percent = min(1.0, percent_from - 0.0)
            hub.light.on(Color.RED)
            #print("DOWN")
        else:
            percent = 1
            hub.light.on(Color.BLUE)

        robot.drive((speed * percent) * direction, heading * -kp * direction)

        if i > 50:
            i = 0
            gc.collect()
        else:
            i = i + 1

    # Stop and hold the engines
    motorL.hold()
    motorR.hold()

    # ----------------------------------------------------------------------------------------------------------------------------------
    hub.light.on(Color.CYAN)

    if speed < 600:
        wait(60)
    else:
        wait(100)

    # Calibrate stopping error
    gc.collect()
    kp = 20
    dist_kp = 10
    dist_tolerance = 4
    tolerance = 0.4
    i = 0
    h = 0

    watch = StopWatch()
    while (abs(heading) > tolerance or abs(((motorR.angle() + motorL.angle()) / 2) - distance) > dist_tolerance or h < 6) and watch.time() < 1200:
        heading = hub.imu.heading()
        robot.drive(-(((motorR.angle() + motorL.angle()) / 2) - distance) * dist_kp, heading * -kp)
        if i > 150:
            i = 0
            gc.collect()
        else:
            i = i + 1
        h = h + 1

    hub.light.on(Color.GREEN)
    motorL.hold()
    motorR.hold()

    gc.enable()

def turn_gyro(angle, speed = baseSpeed, ramp_up=10, ramp_down=40):
    global baseAngle
    target_angle = baseAngle + angle
    baseAngle = target_angle
    # Reset angles
    motorL.reset_angle(0)
    motorR.reset_angle(0)

    speed = speed * 0.15

    track_dist = 10
    kp = 5.5
    i = 0

    start_heading = hub.imu.heading()
    total_angle = abs(target_angle - start_heading)

    # Calculate if we use the built in acceleration profile or to split the drive into 2
    if ramp_up + ramp_down > total_angle:
        print("ROUTE TOO SHORT")
        ramp_up = total_angle / 2
        ramp_down = ramp_up

    if ramp_up == 0:
        ramp_up = 1
    if ramp_down == 0:
        ramp_down = 1

    if target_angle < start_heading:
        direction = 1
    else:
        direction = -1

    # ----------------------------------------------------------------------------------------------------------------------------------
    gc.collect()
    gc.disable()
    heading = hub.imu.heading()
    while abs(heading - target_angle) > track_dist:
        # Get the direction
        heading = hub.imu.heading()
        turned = abs(heading - start_heading)
        remaining = abs(heading - target_angle)

        # Calculate the percentage of the track left in the braking and acceleration zones
        percent_to = abs(turned / ramp_up)
        percent_from = abs(remaining / ramp_down)

        # Decide if we are ramping up, slowing down or staying at constant velocity
        if turned < ramp_up and percent_to < 1:
            percent = min(1.0, percent_to + 0.2)
            hub.light.on(Color.GREEN)
            #print("UP")
        elif remaining < ramp_down and percent_from < 1:
            percent = max(0.2, min(1.0, percent_from))
            hub.light.on(Color.RED)
            #print("DOWN")
        else:
            percent = 1
            hub.light.on(Color.BLUE)

        robot.drive(0, direction * -kp * percent * speed)

        if i > 50:
            i = 0
            gc.collect()
        else:
            i = i + 1

    # Stop and hold the engines
    motorL.hold()
    motorR.hold()

    # ----------------------------------------------------------------------------------------------------------------------------------
    hub.light.on(Color.CYAN)

    if speed > 400:
        wait(60)
    else:
        wait(100)

    # Calibrate stopping error
    gc.collect()
    kp = 18
    tolerance = 0.36
    i = 0
    h = 0

    watch = StopWatch()
    while (abs(heading - target_angle) > tolerance or h < 6) and watch.time() < 1200:
        heading = hub.imu.heading()
        robot.drive(0, (heading - target_angle) * -kp)
        if i > 250:
            i = 0
            gc.collect()
        else:
            i = i + 1
        h = h + 1

    hub.light.on(Color.GREEN)
    motorL.hold()
    motorR.hold()

    gc.enable()


def moduleL(angle, speed):
    leftEngine.run_angle(speed, angle)
    wait(400)

def moduleR(angle, speed):
    rightEngine.run_angle(speed, angle)
    wait(400)

gc.collect()

move_cm(10, 800)
turn_gyro(-90, 500)
turn_gyro(-180, 200)
turn_gyro(20, 200)

print("Exit")
