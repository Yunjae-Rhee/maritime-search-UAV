from dronekit import connect, VehicleMode
import time

# Mac USB 연결 포트
connection_str = "/dev/tty.usbmodem1201"   # ✅ 여기 수정
baud_rate = 115200

print(f"Connecting to Pixhawk on {connection_str}...")
vehicle = connect(connection_str, baud=baud_rate, wait_ready=True)

# 상태 출력
print("Autopilot version:", vehicle.version)
print("Battery:", vehicle.battery)
print("Is Armable?:", vehicle.is_armable)
print("Mode:", vehicle.mode.name)

# ARM 시도
print("Arming motors...")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)
print("✅ Vehicle is ARMED")

time.sleep(5)

vehicle.armed = False
print("✅ Vehicle DISARMED")

vehicle.close()