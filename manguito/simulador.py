import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Función de transferencia de un sistema de control
def system(Kp, Ki, Kd):
    # Parámetros del sistema
    tau = 5.0
    num = [Kp]
    den = [tau, 1]

    # Controlador PID
    numc = [Kd, Kp, Ki]
    denc = [1, 0]

    sys = control.TransferFunction(num, den)
    sysc = control.TransferFunction(numc, denc)

    # Sistema en lazo cerrado
    sys_cl = control.feedback(sys * sysc, 1)
    time = np.linspace(0, 20, 1000)
    t, y = control.step_response(sys_cl, time)
    return t, y

# Valores iniciales de los parámetros
Kp = 1.0
Ki = 0.1
Kd = 0.01

# Simulación del sistema con parámetros iniciales
t, y = system(Kp, Ki, Kd)

# Visualización de la respuesta en el tiempo
plt.plot(t, y)
plt.xlabel('Tiempo')
plt.ylabel('Respuesta')
plt.title('Respuesta del sistema con PID')
plt.show()
