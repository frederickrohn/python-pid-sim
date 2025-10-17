import numpy as np
import matplotlib.pyplot as plt

def plot_output(t, r, y, title="Output vs Setpoint"):
    plt.figure()
    plt.plot(t, r, label="setpoint")
    plt.plot(t, y, label="output")
    plt.xlabel("time (s)")
    plt.ylabel("value")
    plt.title(title)
    plt.legend()
    plt.show()


def plot_control(t, u, title="Control Signal"):
    plt.figure()
    plt.plot(t, u, label="u (control)")
    plt.xlabel("time (s)")
    plt.ylabel("control effort")
    plt.title(title)
    plt.legend()
    plt.show()

# this is our plant, the "thing" we are controlling
# this code is a model of how our plant responds to input
# this is a first order system whose response is a smooth exponential code
# similar to how temperature slowly changes in a room
# assuming a thermostat: y would be temperature, u is the control inptut
# tau is the time constant that tells you how slugish it is, and K is a gain (how strongly it reacts)
def first_order_plant_step(y_prev, u, tau, K, dt):
    """One Euler step of y_dot = (-y + K*u)/tau."""
    return y_prev + dt * ((-y_prev + K * u) / tau)

class PID:
    # PID controller
    def __init__(self, Kp, Ki, Kd, u_min=-np.inf, u_max=np.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.u_min = u_min
        self.u_max = u_max
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error, dt):
        # proportional
        P = self.Kp * error

        # integral (accumulate)
        self.integral += error * dt
        I = self.Ki * self.integral

        # derivative (slope of error)
        D = self.Kd * (error - self.prev_err) / dt

        # raw control
        u = P + I + D

        # clamp (actuator limits)
        if u > self.u_max:
            u = self.u_max
        elif u < self.u_min:
            u = self.u_min

        # remember for next step
        self.prev_err = error
        return u


# ---- simulation settings ----
T = 8.0          # total time (seconds)
dt = 0.001       # step size
N = int(T / dt)  # number of steps
t = np.linspace(0.0, T, N)

# step setpoint: 0 -> 1 at t=0
# this is just an array of 1s meaning our target is at position 1 the whole time
# it's called step setpoint because it "steps" up to 1 at time zero - although, we never really see the part before it becomes 1
r = np.ones(N)

# plant params
tau = 0.7
Kplant = 1.0

# storage
# to plot later, we save arrays for y (output), u(pid command), and e(error/gap from target)
y = np.zeros(N)     # output
u = np.zeros(N)     # control
e = np.zeros(N)     # error

# ---- make a PID controller ----
pid = PID(Kp=2.0, Ki=1.2, Kd=0.1, u_min=-10.0, u_max=10.0)

# ---- run the simulation ----
for k in range(1, N):
    # error between target and last output
    e[k] = r[k] - y[k-1]

    # controller decides control effort
    u[k] = pid.update(e[k], dt)

    # plant responds to control effort
    y[k] = first_order_plant_step(y[k-1], u[k], tau=tau, K=Kplant, dt=dt)

# ---- plot results ----
plot_output(t, r, y, title="PID: step response")
plot_control(t, u, title="PID: control effort")


