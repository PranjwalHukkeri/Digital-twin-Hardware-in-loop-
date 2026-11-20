"""
Baseline Simulation Configuration and Initial AEB Algorithm Parameters.

Values distinguish:
1. Initial literature-based parameters (adopted for initial model, subject to sensitivity analysis)
2. Simulation software assumptions
3. Values TBD (To Be Determined)
"""

# ==============================================================================
# 1. INITIAL LITERATURE-BASED CONTROLLER PARAMETERS
# ==============================================================================
# Time-To-Collision (TTC) thresholds in seconds
TTC_WARNING = 4.00  # seconds - Warning state threshold
TTC_LEVEL_1 = 3.00  # seconds - Level 1 AEB deceleration threshold
TTC_LEVEL_2 = 2.25  # seconds - Level 2 AEB deceleration threshold
TTC_LEVEL_3 = 1.75  # seconds - Level 3 AEB deceleration threshold

# Desired deceleration levels in m/s^2 (negative values indicate deceleration)
A_LEVEL_1 = -2.0  # m/s^2 - Level 1 braking
A_LEVEL_2 = -4.0  # m/s^2 - Level 2 braking
A_LEVEL_3 = -6.0  # m/s^2 - Level 3 emergency braking

# ==============================================================================
# 2. INITIAL SOFTWARE SIMULATION CONDITIONS
# ==============================================================================
DT = 0.05  # seconds - Discrete simulation timestep

# Default scenario initial conditions
INITIAL_EGO_SPEED = 1.0  # m/s - Ego vehicle initial speed
INITIAL_TARGET_SPEED = 0.0  # m/s - Target vehicle initial speed
INITIAL_DISTANCE = 5.0  # m - Initial relative distance to target

# ==============================================================================
# 3. ENGINEERING DESIGN PARAMETERS (TEMPORARY FOR DISCRETE EXECUTION)
# ==============================================================================
# Minimum active AEB braking dwell duration in seconds.
# Note: Engineering design parameter for anti-chattering stability. Not literature-derived.
MIN_BRAKE_DURATION = 0.20  # seconds (temporary default value for discrete-time execution)

# ==============================================================================
# 4. VALUES STILL TBD (TO BE DETERMINED IN LATER PHASES)
# ==============================================================================
# Vehicle dynamics parameters (TBD):
# - Vehicle mass (kg)
# - Wheel radius (m)
# - Wheelbase (m)
# - Tire friction coefficient (mu)
# - Maximum brake torque / force (N)
# - Actuator lag / dynamic response time (s)
# - Sensor noise and sensor latency (s)
