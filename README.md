# Reaction Wheel Inverted Pendulum Control

## Contributors

- Enzo Oliveira Batista (https://github.com/EnzoBat17)
- João Vitor Faria (https://github.com/jvfaria-dev)
- Vitor Oliveira Batista (https://github.com/Vitor-Oliveira17)

Repository dedicated to the dynamic modeling, digital simulation (**MATLAB/Simulink**), and real-time embedded implementation (**ESP32 / ESP-IDF**) of three discrete-time control strategies applied to a **Reaction Wheel Inverted Pendulum**.

Project developed at the **Faculty of Electrical Engineering (FEELT)**, **Federal University of Uberlândia (UFU)**.

## 📁 Repository Structure

The project is organized into three standalone directories, each containing its respective MATLAB parameterization/design scripts, Simulink simulation diagrams, and C embedded firmware for the ESP32:

```text
├── LQR/               # Discrete Linear Quadratic Regulator (dlqr) + Simulation + ESP32 Firmware
├── Pole Placement/    # Discrete State-Space Pole Placement (place) + Simulation + ESP32 Firmware
└── PID/               # Discrete Parallel PID (Tustin) with Velocity Feedback (Kw) + Simulation + ESP32 Firmware
```

## ⚙️ Dynamic Modeling and System Parameters

The electromechanical system was modeled using the **Newton-Euler** formulation and linearized around the unstable upright equilibrium point ($\theta_{eq} = 0\text{ rad}$), considering the state vector:

$$
x(t) = \begin{bmatrix} \theta(t) \\ \dot{\theta}(t) \\ \omega_{rel}(t) \end{bmatrix}
$$

Where:

* $\theta$: Pendulum tilt angle relative to the vertical $[\text{rad}]$
* $\dot{\theta}$: Pendulum angular velocity $[\text{rad/s}]$
* $\omega_{rel}$: Relative angular velocity of the reaction wheel with respect to the pendulum $[\text{rad/s}]$
* $u(t)$: Voltage applied to the DC motor terminals $[\text{V}]$

### Physical and Electromechanical Parameters

| Parameter | Symbol | Value | Unit | Description |
| :--- | :---: | :---: | :---: | :--- |
| Pendulum rod mass | $M_h$ | $0.038$ | $\text{kg}$ | Uniform rectangular bar |
| Reaction wheel mass | $M_r$ | $0.179$ | $\text{kg}$ | Thick ring geometry ($R_{ext}=0.07\text{ m}$, $R_{int}=0.055\text{ m}$) |
| Motor mass | $M_m$ | $0.213$ | $\text{kg}$ | DC Motor AK555/390ML12S18200C ($R_{motor}=0.0178\text{ m}$) |
| Rod length | $L_h$ | $0.15144$ | $\text{m}$ | Center of mass at $L_b = L_h / 2$ |
| Pivot-to-wheel distance | $L_r$ | $0.10$ | $\text{m}$ | Distance from the base pivot to the motor/wheel center |
| Nominal voltage | $V_{nom}$ | $12.0$ | $\text{V}$ | Actuator saturation bounded to $[-12\text{ V}, +12\text{ V}]$ |
| Armature resistance | $R_m$ | $0.59$ | $\Omega$ | Estimated for 555-series DC motor ($12\text{ V}$) |
| No-load speed | $RPM_{no\_load}$ | $18200$ | $\text{RPM}$ | $\omega_0 \approx 1905.9\text{ rad/s}$ |
| Motor constants | $k_e = k_t$ | $0.0063$ | $\text{V}\cdot\text{s/rad}$ | Back-EMF and electromagnetic torque constants |
| Sampling period | $T_s$ | $0.001$ | $\text{s}$ | Digital control loop running at $1\text{ kHz}$ (ZOH) |

## 🧠 Implemented Control Strategies

### 1. `LQR/` — Discrete Linear Quadratic Regulator

Minimizes the discrete-time quadratic cost function $J = \sum_{k=0}^{\infty} (x[k]^T Q x[k] + u[k]^T R u[k])$ by solving the Discrete-Time Algebraic Riccati Equation (`dlqr` in MATLAB):

* **Weighting matrices:**

  $$
  Q = \begin{bmatrix} 42 & 0 & 0 \\ 0 & 0.68 & 0 \\ 0 & 0 & 0.001 \end{bmatrix}, \quad R = [1]
  $$

* **Control law:** $u[k] = \text{sat}(r[k] - K_d x[k])$
* **Practical note:** Although it achieves fast convergence in simulation, the high state gain on $\theta$ demands initial current peaks that can trigger the overcurrent protection limit ($\approx 5\text{ A}$) of standard benchtop DC power supplies.

### 2. `Pole Placement/` — State-Space Pole Placement

Directly assigns the closed-loop eigenvalues $(A_d - B_d K_d)$ to achieve a smooth, well-damped transient response without overloading the power supply:

* **Desired continuous poles:** $\lambda = [-1,\ -4,\ -5]\ \text{rad/s}$
* **Discrete poles ($z_i = e^{\lambda_i T_s}$):** $z = [0.99900,\ 0.99601,\ 0.99501]$
* **Discrete feedback gains (`place`):**

  $$
  K_d = \begin{bmatrix} -52.2083 & -4.6407 & -0.2471 \end{bmatrix}
  $$

### 3. `PID/` — Discrete Parallel PID + Wheel Velocity Feedback ($K_w$)

Classical architecture discretized via the **trapezoidal (Tustin) method** with a first-order low-pass derivative filter ($N = 100$) and a parallel wheel angular velocity feedback branch ($K_w$) for active braking and angular momentum management:

* **Control law:**

  $$
  e[k] = r[k] - \theta[k]
  $$

  $$
  u[k] = \text{sat}\Big(u_p[k] + u_i[k] + u_d[k] - K_w \omega_{rel}[k]\Big)
  $$

* **Tuned gains:** $K_p = -109.0$, $K_i = 0.0$ (integral action disabled to prevent windup), $K_d = -12.0$, $K_w = -0.47$.

## 🔧 Hardware and Embedded Firmware (ESP32)

The firmware is written in C using the **ESP-IDF** framework and a multitasking **FreeRTOS** architecture:

* **Microcontroller:** ESP32 (control loop pinned to Core 0 at $1\text{ kHz}$ / $T_s = 1\text{ ms}$; UART telemetry/logging task decoupled at low priority via a `Queue`).
* **Sensors (I2C Bus):**
  * **IMU (MPU6050 — address `0x68`):** Measures pendulum tilt angle $\theta$ and angular velocity $\dot{\theta}$.
  * **Magnetic Encoder (AS5600 — address `0x36`):** Directly measures the relative wheel angular velocity $\omega_{rel}$, filtered via an exponential moving average ($\alpha = 0.2$, $f_c \approx 36\text{ Hz}$).
* **Actuation:** **BTS7960** H-bridge driven by PWM (mapping $[-12\text{ V}, +12\text{ V}]$ to a duty cycle of $[-255, +255]$).
* **Operating Modes (`SEC_MODE` in `main.c`):**
  * `SEC_MODE_BALANCE`: Closed-loop balancing mode (automatically arms after holding $|\theta| < 2^\circ$ for $500\text{ ms}$; disarms upon fall beyond $30^\circ$, IMU failure, or continuous actuator saturation for $1\text{ s}$).
  * `SEC_MODE_SENSORS`: Sensor readout and calibration with the motor disabled.
  * `SEC_MODE_MOTOR_TEST`: Open-loop motor and encoder test routine.
  * `SEC_MODE_PIN_TEST`: Static logic-level test on the H-bridge control pins (`GPIO 25, 26, 32, 33`).

## 🚀 Getting Started

### Simulation (MATLAB / Simulink)

1. Open MATLAB and navigate to the desired controller folder (`LQR/`, `Pole Placement/`, or `PID/`).
2. Run the `.m` parameterization script to load the discrete matrices $A_d, B_d, C_d, D_d$ and controller gains into the Workspace.
3. Open and run the corresponding `.slx` model in Simulink (default initial condition: $\theta_0 = 12^\circ$).

### Embedded Firmware (ESP-IDF)

1. Navigate to the firmware directory inside the chosen controller folder.
2. Set the target chip and build the project:

   ```bash
   idf.py set-target esp32
   idf.py build
   ```

3. Flash the firmware and open the serial monitor (`115200 baud`), holding the pendulum steady in the upright position during the calibration countdown:

   ```bash
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

## 👥 Authors

* **Enzo Oliveira Batista** — `12411ECP009`
* **João Vitor Faria Silva** — `12321ECP013`
* **Vítor Oliveira Batista** — `12411ECP008`

## 📚 References

1. BELASCUEN, G.; AGUILAR, N. *Design, Modeling and Control of a Reaction Wheel Balanced Inverted Pendulum*. Instituto Tecnológico de Buenos Aires (ITBA), Buenos Aires, Argentina, 2018.
2. ÅSTRÖM, K. J.; WITTENMARK, B. *Computer-Controlled Systems: Theory and Design*. 3rd ed. Upper Saddle River: Prentice Hall, 1997.
