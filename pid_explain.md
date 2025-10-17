# 🤖 PID Controller Simulation

## 🧠 The Big Idea
$$
u(t) = K_p \, e(t) \;+\; K_i \int_0^t e(\tau)\,d\tau \;+\; K_d \frac{d e(t)}{dt}
$$

A PID controller is a way to automatically steer a system toward a goal.
- Goal (setpoint): what you want (e.g., 72°F room temp, a robot moving towards a target 10m away).
- Measurement (output): what you have (e.g., 70°F).
- Error = Goal − Output.
    - The controller constantly looks at the error and decides how hard to push on the system.

## 🧑‍🍳 The 3 Ingredients
P = Proportional: reacts to the error right now (CURRENT)
- Big error → big correction.
    - Think: steering back onto the road.

I = Integral: reacts to the history of error (PAST)
- If you’ve been off-target for a while, it keeps piling on correction.
    - Think: if you always drift right, I “remembers” and biases your steering left.

D = Derivative: reacts to the slope of error (FUTURE)
- If you’re correcting too fast, it eases off so you don’t overshoot.
    - Think: braking smoothly before a stop sign.

Together, PID = P + I + D (P,I,and D are multiplied by constants Kp, Ki, and Kd respectively so we can weight them differently based on the problem we hope to solve) → balanced, smooth correction. We also might clamp u to certain value ranges to match the realistic possibilities (ie. a car cannot accelarate any faster than it's physically capable of doing - we can't press down on the gas any harder than fully being pressed)

## 🪴 What is a plant? 
The plant is just the thing being controlled (the system you’re pushing on).
- In our simulation, the plant is a simple “sluggish” model, a first-order system:
$$
\dot{y}(t) = \frac{-y(t) + K \cdot u(t)}{\tau}
$$

## 🛠️ How to use the file pid.py
* Run the file using `python3 pid.py` → you’ll see two plots
* Tweak Kp, Ki, Kd values in the PID constructor.
    - Raise Kp: reacts faster but may overshoot.
    - Add Ki: eliminates steady offset.
    - Add Kd: smooths out/dampens overshoot/oscillation
        - As you'll see, setting a large value for Kd can cause u to blow up & oscillate like crazy - this is like overcorrecting when steering a car and causing it to fishtail, so you have to be really careful with this term
        - In real life, engineers either keep it small or add a low-pass filter to stop this from happening

## 🗺️ Limitations and Future Directions
PID controllers are simple and effective, but they have limits:

- Gains ($K_p, K_i, K_d$) must be tuned by **trial and error** 🎛️  
- They only look at the **error**, not the full physics of the system - ie. they treat the system itself like a black box
- Performance can drift if conditions change (extra load, hills, friction)

👉 The next step is **model-based control** 📐:  
Instead of guessing gains, you build a **mathematical model** of the system and design the controller directly from it.

**Analogy:**  
- PID = driving a car by *feel*, adjusting the gas pedal until it’s smooth
- Model-based = driving with the *physics textbook in hand*, calculating exactly how the car will respond using your knowledge of drag, throttle, engine rpm, etc. before you press the gas

In other words, before you even test/simulate the system, you already have an idea of how it will respond ahead of time. This is the next step in control systems theory. 

The **first model-based approach** you should learn is **state-space control**:  
- It represents systems with internal **states** (e.g. position, velocity) and matrices $(A, B, C, D)$ - a lot of linear algebra
- We start here because it’s the framework all modern model-based controls sit on. once you can read and write state-space equations everything else (observers, optimal control, Kalman filters, MPC) builds naturally from it.
- Here's a good intro to that subject :)
    - [Intro to state-space control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html)
