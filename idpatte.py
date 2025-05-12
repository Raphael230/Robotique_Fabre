import pypot.dynamixel
import time 

# Remplace ici par le bon port
PORT = 'COM8' # ou '/dev/ttyUSB0'

dxl_io = pypot.dynamixel.DxlIO("COM8", baudrate=1000000)
ids = dxl_io.scan()
print("Moteurs détectés :", ids)

for motor_id in ids:
    print(f"Test moteur {motor_id}")
    dxl_io.set_goal_position({motor_id: 30})
    time.sleep(1)
    dxl_io.set_goal_position({motor_id: -30})
    time.sleep(1)
    dxl_io.set_goal_position({motor_id: 0})
    print(f"fin du test {ids}")
    time.sleep(3)
    