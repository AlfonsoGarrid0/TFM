import serial, struct, numpy as np
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === Parámetros ===
PORT   = 'COM3'
FS     = 256
BUF_S  = 2    # segundos de buffer
PKT_SZ = 80   # 16 floats (64B) + 16 flags (16B)

# === Setup serial y buffers ===
ser    = serial.Serial(PORT, timeout=0.01)
N      = FS * BUF_S
buffer = np.zeros(N, dtype=float)
stim   = np.zeros(N, dtype=np.uint8)
t      = np.linspace(-BUF_S, 0, N)

# Para acumular TODO el histórico
hist_accel = []
hist_stim  = []

# === Ventana PyQtGraph ===
app  = QtWidgets.QApplication([])
win  = pg.GraphicsLayoutWidget(show=True, title="z_accel @256 Hz")
plot = win.addPlot()
curve= plot.plot(pen='y')
plot.setLabel('left',   'Aceleración Z', units='g')
plot.setLabel('bottom', 'Tiempo',         units='s')
plot.setYRange(-4, 4)

accumulator = bytearray()

def update():
    global accumulator, buffer, stim, hist_accel, hist_stim

    # 1) Leer bytes entrantes
    incoming = ser.read(ser.in_waiting or 1)
    accumulator.extend(incoming)

    updated = False
    # 2) Procesar todos los paquetes completos
    while len(accumulator) >= PKT_SZ:
        pkt = accumulator[:PKT_SZ]
        accumulator = accumulator[PKT_SZ:]

        vals  = struct.unpack('<16f', pkt[:64])
        flags = list(pkt[64:80])     # 16 bytes directamente 0,1 o 2

        # 3) Mover buffer circular
        buffer = np.roll(buffer, -16)
        stim   = np.roll(stim,   -16)
        buffer[-16:] = vals
        stim[-16:]   = flags

        # 4) Acumular histórico completo
        hist_accel.extend(vals)
        hist_stim .extend(flags)

        updated = True

    if updated:
        # 5) Redibujar solo la curva de aceleración
        curve.setData(t, buffer)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(int(1000/FS))

# === Guardado al cerrar ===
def on_exit():
    # Convertir a arrays
    acc = np.array(hist_accel, dtype=float)
    st  = np.array(hist_stim,  dtype=int)

    if acc.size == 0:
        print("No hay datos que guardar.")
        return

    times = np.arange(acc.size) / FS   # escala de tiempo

    # 1) Guardar CSV con NumPy
    data = np.column_stack((times, acc, st))
    header = 'time_s,z_accel,stim_flag'
    np.savetxt('test06_09_5.csv', data, delimiter=',', header=header, comments='')
    print("Guardado correctamente")

    # 2) Plot offline con matplotlib y guardar PNG
    plt.figure(figsize=(10,4))
    plt.plot(times, acc, color='k', linewidth=0.8)

    # Dibujar franjas
    in_block = False
    curr = None        # flag del bloque actual (1 o 2)
    start_i = None     # índice de inicio del bloque

    for i, flag in enumerate(st.tolist()):
        if flag != 0 and not in_block:
            # Abrimos un bloque nuevo
            start_i = i
            curr = flag
            in_block = True
        elif in_block and flag != curr:
            # Cerramos el bloque actual
            end_i = i
            color = 'red' if curr == 1 else 'blue'
            plt.axvspan(start_i/FS, end_i/FS, color=color, alpha=0.3, lw=0)
            in_block = False
            # Si el nuevo flag es distinto de cero, abrimos el siguiente bloque inmediatamente
            if flag != 0:
                start_i = i
                curr = flag
                in_block = True

    # Si queda un bloque abierto hasta el final de los datos, ciérralo
    if in_block and start_i is not None and curr is not None:
        plt.axvspan(start_i/FS, acc.size/FS, color=('red' if curr == 1 else 'blue'), alpha=0.3, lw=0)

    plt.xlabel('Tiempo (s)')
    plt.ylabel('Aceleración Z (g)')
    plt.title('Aceleración Z con Estimulación')
    plt.tight_layout()
    plt.savefig('accel_with_stim.png', dpi=150)
    print("Guardado png correctamente")

app.aboutToQuit.connect(on_exit)
app.exec_()
