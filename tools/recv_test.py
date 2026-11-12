from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
m.wait_heartbeat()
print("Heartbeat da")
while True:
    msg = m.recv_match(blocking=True)
    print(msg.get_type())
