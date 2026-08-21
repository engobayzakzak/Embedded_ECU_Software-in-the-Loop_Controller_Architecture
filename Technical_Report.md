# Comprehensive Technical Report: Embedded Software-in-the-Loop (SIL) Controller Architecture

---

## 1. System Mathematical Modeling

### 1.1 Second-Order Continuous Plant Dynamics

The actuator plant is modeled as a continuous second-order electromechanical system:
$$J \ddot{\theta}(t) + B \dot{\theta}(t) = K_t \cdot u(t)$$

Normalizing with respect to rotor inertia $J$:
$$\ddot{\theta}(t) = \alpha \cdot u(t) - \beta \cdot \dot{\theta}(t)$$

Where:

- $\alpha = \frac{K_t}{J} = 25.0\text{ rad}/(\text{s}^2 \cdot \text{unit})$ (Actuator Torque Gain)
- $\beta = \frac{B}{J} = 2.0\text{ s}^{-1}$ (Viscous Friction Coefficient)
- $u(t) \in [-1.0, +1.0]$ (Normalized Control Input)

### 1.2 Discrete Numerical Integration (Forward Euler)

For digital implementation inside the $1000\text{ Hz}$ control task ($\Delta t = 0.001\text{ s}$):
$$v[k+1] = v[k] + \left( 25.0 \cdot u[k] - 2.0 \cdot v[k] \right) \Delta t$$
$$\theta[k+1] = \theta[k] + v[k+1] \Delta t$$

---

## 2. Discrete PID Formulation & Anti-Windup Strategy

### 2.1 Parallel Form with Derivative on Measurement

To prevent derivative kick during step changes in setpoint $\theta_r[k]$, differentiation is applied directly to plant position measurement $\theta[k]$:

$$e[k] = \theta_r[k] - \theta[k]$$
$$P[k] = K_p \cdot e[k]$$
$$I[k] = I[k-1] + K_i \cdot e[k] \cdot \Delta t$$

### 2.2 First-Order Derivative Low-Pass Filter

To attenuate high-frequency encoder quantization noise:
$$\frac{d\theta}{dt}_{\text{filt}}[k] = \frac{d\theta}{dt}_{\text{filt}}[k-1] + \alpha_d \left( -\frac{\theta[k] - \theta[k-1]}{\Delta t} - \frac{d\theta}{dt}_{\text{filt}}[k-1] \right)$$

Where $\alpha_d = \frac{\Delta t}{\tau + \Delta t}$, with filter time constant $\tau = 10\Delta t$.

### 2.3 Conditional Integration Clamping (Anti-Windup)

$$I[k] = \text{clamp}\left(I[k], -u_{\max}, +u_{\max}\right)$$
$$u[k] = \text{clamp}\left(P[k] + I[k] + D[k], -u_{\max}, +u_{\max}\right)$$

---

## 3. Communication Acceptance Filtering Mathematics

The virtual CAN driver implements standard hardware bitwise mailbox acceptance masking:

$$\text{Frame Accepted} \iff (\text{CAN\ID} \land \text{MASK}) == (\text{FILTER\ID} \land \text{MASK})$$

For Filter Bank 0:

- $\text{FILTER\ID} = 0x100$ (`0001 0000 0000_b`)
- $\text{MASK} = 0x700$ (`0111 0000 0000_b`)
- **Accepted ID Range:** $0x100 \le \text{ID} \le 0x1FF$ (All Command and Telemetry Packets Accepted; peripheral noise dropped at zero CPU cost).

---

## 4. Hardware-Software Co-Design Specification

### 4.1 Galvanic Isolation Topology

To protect the MCU logic domain against ground bounce and high-voltage back-EMF inductive kickbacks:

- Digital Isolators: Silicon Labs Si8621 ($5\text{ kV}_{\text{RMS}}$ isolation rating).
- Split Grounding: Dedicated isolated grounds ($\text{GND}_{\text{LOGIC}}$ and $\text{GND}_{\text{POWER}}$) bridged at a single star point via a ferrite bead.

### 4.2 Thermal & Current Budget

- Maximum continuous RMS phase current: $15\text{ A}$.
- Sense Shunt Power Dissipation: $P = I^2 R = (15\text{ A})^2 \times 0.005\,\Omega = 1.125\text{ W}$ (Rated for $3\text{ W}$, 2512 package).
