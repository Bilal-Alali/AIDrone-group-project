import math, time
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

RES = 0.05                    # m pro Zelle
SIZE = 400                    # 400 x 400 Zellen = 20 x 20 m
L = SIZE * RES / 2
grid = np.zeros((SIZE, SIZE)) # Log-Odds: >0 belegt, <0 frei, 0 unbekannt

def to_cell(x, y):
    return int(round(x / RES + SIZE / 2)), int(round(y / RES + SIZE / 2))

def in_grid(i, j):
    return 0 <= i < SIZE and 0 <= j < SIZE

def update(px, py, yaw, d_m, ori, max_m, min_m):
    if d_m < min_m:
        return
    a = yaw + math.radians(ori * 45)      # 0=vorn, 2=rechts, 6=links
    hit = d_m < max_m
    d = min(d_m, max_m)
    for k in range(int(d / (RES / 2))):   # Strahl: Zellen davor sind frei
        r = k * RES / 2
        i, j = to_cell(px + r * math.cos(a), py + r * math.sin(a))
        if in_grid(i, j):
            grid[i, j] = max(grid[i, j] - 0.4, -5)
    if hit:                                # Endpunkt: Zelle ist belegt
        i, j = to_cell(px + d * math.cos(a), py + d * math.sin(a))
        if in_grid(i, j):
            grid[i, j] = min(grid[i, j] + 0.9, 5)

m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
m.wait_heartbeat()
print("Verbunden")

px = py = yaw = 0.0
plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(grid, origin="lower", extent=[-L, L, -L, L],
                vmin=-5, vmax=5, cmap="gray_r")
dot, = ax.plot([0], [0], "ro")
ax.set_xlabel("Ost [m]")
ax.set_ylabel("Nord [m]")

last = 0
while plt.fignum_exists(fig.number):
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "LOCAL_POSITION_NED":
            px, py = msg.x, msg.y
        elif t == "ATTITUDE_QUATERNION":
            w, x, y, z = msg.q1, msg.q2, msg.q3, msg.q4
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        elif t == "DISTANCE_SENSOR":
            update(px, py, yaw, msg.current_distance / 100.0, msg.orientation,
                   msg.max_distance / 100.0, msg.min_distance / 100.0)
    if time.time() - last > 0.2:
        img.set_data(grid)
        dot.set_data([py], [px])
        fig.canvas.draw_idle()
        plt.pause(0.01)
        last = time.time()
    else:
        time.sleep(0.005)
