from time import time
from statistics import fmean
from itertools import islice
from collections import deque

from serial import Serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_BUFFER = 10  # maximum number of data points to keep in view (autoscroll to right).
NOISE_THRESHOLD = 0.04
ROTATION_TIMEOUT = 2
CURVE_AREA_THRESHOLD = 200  # chosen by trial and error

# Setup serial
serial = Serial(port="/dev/ttyUSB0", baudrate=1000000)


def read_values(io: Serial = serial) -> list[float]:
    return [float(item) for item in io.readline().decode().split()]


for _ in range(10):
    read_values()

# Setup initial data
try:
    z_buffer = deque([read_values()[2]] * MAX_BUFFER, maxlen=MAX_BUFFER)  # Raw z data (with noise filter).
    z_avg_buffer = z_buffer.copy()  # Simple moving average of `z_buffer`.
    force_buffer = deque([read_values()[-1]] * MAX_BUFFER, maxlen=MAX_BUFFER)
    force_avg_buffer = force_buffer.copy()
except ValueError:
    z_buffer = deque([0] * MAX_BUFFER, maxlen=MAX_BUFFER)
    z_avg_buffer = z_buffer.copy()
    force_buffer = deque([0] * MAX_BUFFER, maxlen=MAX_BUFFER)
    force_avg_buffer = force_buffer.copy()
time_buffer = deque([time()] * MAX_BUFFER, maxlen=MAX_BUFFER)  # Timestamps for items in z_buffer.
slope_buffer = deque([0.0] * MAX_BUFFER, maxlen=MAX_BUFFER)  # Slope for items in z_avg_buffer.
intercept_time_buffer = deque([0.0] * 3, maxlen=3)  # Timestamps at which an x-intercept occurred.
rpm = 0
force = 0

# Setup plot
fig, (ax1, ax2) = plt.subplots(2)
RPM_LABEL = fig.text(.15, .85, f"RPM: {rpm}", fontsize=15)
LOAD_LABEL = fig.text(.15, .4, f"Force (g): {force}", fontsize=15)


def animate(_interval):
    process_data()
    update_plot()


def process_data():
    global rpm
    global force

    current_time = time()

    try:
        a_x, a_y, a_z, load = read_values(serial)
    except ValueError:
        # rotating at high speeds probably causes loose connections and therefore errors reading from the sensor.
        # (which should be fixed when we solder it all)
        append_z_value(z_buffer[-1], current_time)
        append_load_value(force_buffer[-1])
        return

    # Noise filter
    if abs(a_z - z_buffer[-1]) < NOISE_THRESHOLD:
        append_z_value(z_buffer[-1], current_time)
        append_load_value(load)
        return

    append_z_value(a_z, current_time)
    append_load_value(load)

    # An x-intercept occurred
    if not (slope_buffer[-1] * slope_buffer[-2] < 0):
        return
    intercept_time_buffer.append(current_time)

    start_time = intercept_time_buffer.popleft()

    # When we process a rotation, we clear the intercept time buffer with zeroes because
    # otherwise half of the current rotation would count towards the next rotation.
    if start_time == 0:
        return
    intercept_time_buffer.extend([0.0] * 3)

    # We want to ignore tiny oscillations in the z-axis which will cause a slope
    # with a lower magnitude and thus a lower area under the curve for the last
    # n data points as compared to an actual revolution of the pedal which will
    # cause a slope with a larger magnitude.
    if absolute_area_under_curve(slope_buffer, 5) < CURVE_AREA_THRESHOLD:
        rpm = 0
        return

    # `current_time - start_time` is how long a rotation took (in seconds), and so the
    # RPM should be 60 / (t-start), however, that seems to produce unrealistic values,
    # and I'm not sure why, so I've scaled it down for now.
    rpm = int((60 / (current_time - start_time) * 0.3))  # int because nobody cares about them floating points ;)


def update_plot():
    ax1.clear()
    ax2.clear()
    ax1.plot(time_buffer, z_buffer, label="raw", linewidth=1)
    ax2.plot(time_buffer, force_buffer, label="raw")
    # ax1.plot(time_buffer, z_avg_buffer, label="avg")
    # ax1.plot(time_buffer, slope_buffer, label="slope")
    RPM_LABEL.set_text(f"RPM: {rpm}")
    LOAD_LABEL.set_text(f"Force(g): {force}")


def append_z_value(val: float, t: float):
    z_buffer.append(val)
    time_buffer.append(t)
    z_avg_buffer.append(moving_average(z_buffer))
    slope_buffer.append((z_avg_buffer[-1] - z_avg_buffer[-2]) / (time_buffer[-1] - time_buffer[-2]))


def append_load_value(val: float):
    global force
    force = val
    force_buffer.append(val)
    force_avg_buffer.append(moving_average(force_buffer))


def moving_average(data: deque[float], window=2) -> float:
    return fmean(islice(data, MAX_BUFFER - window, MAX_BUFFER))


def absolute_area_under_curve(queue: deque[float], window: int):
    return sum([abs(queue[i]) for i in range(-1, -window - 1, -1)])


# noinspection PyTypeChecker
ani = animation.FuncAnimation(fig, animate, interval=10)
plt.get_current_fig_manager().resize(1920, 1080)
plt.show()
serial.close()
