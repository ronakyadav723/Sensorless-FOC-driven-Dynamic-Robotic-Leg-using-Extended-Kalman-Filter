# Motor electrical properties
Rs = 2.875        # Resistance (Ohms)
Ld = 6e-3         # d-axis inductance (Henry)
Lq = 10.5e-3      # q-axis inductance (Henry)
lambda_m = 0.175  # Flux linkage (Weber)
P = 4             # Pole pairs

# Simulation settings
fsw = 20000       # Switching frequency (Hz)
dt = 1e-5         # Time step (seconds) — matches EKF_BackEMF tuning
Vref = 3.3        # ADC reference voltage (V)
ADC_bits = 12     # ADC resolution (bits)

