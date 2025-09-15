# -*- coding: utf-8 -*-
"""
Sincroniza ACC (CSV) + EMG (MAT) y dibuja franjas de estimulación
sobre ambas gráficas (EMG y aceleración) en un HTML interactivo.

"""

import numpy as np
from scipy.io import loadmat
from scipy.signal import decimate, resample_poly
from pathlib import Path
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ========= RUTAS A FICHEROS =========
ACC_CSV = Path(r"test1.csv")             # columnas: time_s, z_accel, stim_flag
EMG_MAT = Path(r"test1_qua_nostim.mat")     # variables: Data (Nx3), Time (N), SamplingFreq (esc.)

# ====================================
# Utilidades de carga
# ====================================
def load_accel_csv(path: Path):
    arr = np.loadtxt(path, delimiter=",", skiprows=1)
    t    = arr[:, 0].astype(float)
    z    = arr[:, 1].astype(float)
    flag = arr[:, 2].astype(int)
    fs   = 1.0 / np.median(np.diff(t))
    return t, z, flag, fs

def _unwrap(x):
    x = np.array(x).squeeze()
    while isinstance(x, np.ndarray) and x.dtype == object:
        x = x.item()
        x = np.array(x).squeeze()
    return np.array(x)

def load_emg_mat(path: Path):
    mat  = loadmat(path, squeeze_me=True, struct_as_record=False)
    data = _unwrap(mat["Data"])            # esperado: N x 3  (ch1=ECR, ch2=FCR, aux)
    time = _unwrap(mat["Time"]).ravel()    # N
    if "SamplingFreq" in mat:
        fs_emg = float(np.array(mat["SamplingFreq"]).squeeze())
    else:
        fs_emg = 1.0 / np.median(np.diff(time))

    if data.ndim != 2 or data.shape[1] < 3:
        raise ValueError(f"Data debería ser N x 3 (ECR, FCR, AUX). shape={data.shape}")

    ch1 = data[:, 0]  # ECR (extensor)
    ch2 = data[:, 1]  # FCR (flexor)
    aux = data[:, 2]  # trigger/aux
    return time, ch1, ch2, aux, fs_emg

# ====================================
# Detección del bloque activo via AUX
# ====================================
def find_active_block_from_aux_fixed(t, aux_mV, fs, thresh_mV=1000.0, min_seconds=5.0):
    """
    Detecta el bloque principal donde AUX > umbral fijo (en mV).
    Devuelve (start_idx, end_idx) con end exclusivo.
    """
    aux_mV = np.asarray(aux_mV).ravel()
    mask = aux_mV > thresh_mV

    d = np.diff(mask.astype(int), prepend=0, append=0)
    starts = np.where(d == 1)[0]
    ends   = np.where(d == -1)[0]

    if starts.size == 0:
        raise RuntimeError(f"No se detectó AUX alto con umbral {thresh_mV} mV.")

    min_len = int(min_seconds * fs)
    segments = [(s, e) for s, e in zip(starts, ends) if (e - s) >= min_len]
    if not segments:
        raise RuntimeError(f"No hay segmentos con duración >= {min_seconds}s (umbral {thresh_mV} mV).")

    s_best, e_best = max(segments, key=lambda se: se[1] - se[0])
    return s_best, e_best

# ====================================
# Pipeline principal
# ====================================
def main():
    # --- Carga ---
    t_acc, z_acc, stim_flag, fs_acc_est = load_accel_csv(ACC_CSV)
    t_emg, emg_ext, emg_flex, aux, fs_emg = load_emg_mat(EMG_MAT)

    # --- Recorte EMG a su bloque activo por AUX ---
    s_emg, e_emg = find_active_block_from_aux_fixed(
        t_emg, aux, fs_emg, thresh_mV=1000.0, min_seconds=5.0
    )
    t_emg_seg    = t_emg[s_emg:e_emg] - t_emg[s_emg]
    emg_ext_seg  = emg_ext[s_emg:e_emg]
    emg_flex_seg = emg_flex[s_emg:e_emg]

    # --- Sincronización a 256 Hz ---
    FS_ACC = 256
    FS_EMG = int(round(fs_emg))  # típicamente 2048

    # Duraciones
    T_acc     = t_acc[-1] - t_acc[0]
    T_emg_seg = t_emg_seg[-1] - t_emg_seg[0]
    T_common  = min(T_acc, T_emg_seg)

    # Recorte aceleración al tiempo común
    L_acc = int(np.floor(T_common * FS_ACC))
    acc_sync   = z_acc[:L_acc].copy()
    stim_sync  = stim_flag[:L_acc].copy()
    t_sync     = np.arange(L_acc) / FS_ACC

    # Remuestrear EMG a 256 Hz (factor entero si es 2048→256)
    target_fs = FS_ACC
    ratio = FS_EMG / target_fs
    if abs(ratio - round(ratio)) < 1e-9:
        DEC = int(round(ratio))
        emg_ext_ds  = decimate(emg_ext_seg,  DEC, ftype="fir", zero_phase=True)
        emg_flex_ds = decimate(emg_flex_seg, DEC, ftype="fir", zero_phase=True)
    else:
        from math import gcd
        up, down = target_fs, FS_EMG
        g = gcd(up, down)
        emg_ext_ds  = resample_poly(emg_ext_seg,  up//g, down//g)
        emg_flex_ds = resample_poly(emg_flex_seg, up//g, down//g)

    # Igualar longitudes
    L_emg = int(np.floor(T_common * target_fs))
    emg_ext_ds  = emg_ext_ds[:L_emg]
    emg_flex_ds = emg_flex_ds[:L_emg]

    L_min = min(acc_sync.size, emg_ext_ds.size, emg_flex_ds.size, t_sync.size, stim_sync.size)
    acc_sync    = acc_sync[:L_min]
    emg_ext_ds  = emg_ext_ds[:L_min]
    emg_flex_ds = emg_flex_ds[:L_min]
    t_sync      = t_sync[:L_min]
    stim_sync   = stim_sync[:L_min]

    # --- Construcción de franjas a partir de stim_sync ---
    # Creamos bloques contiguos con el mismo valor != 0
    shapes = []
    in_block = False
    curr_val = 0
    start_i = 0

    def color_for(val: int) -> str:
        if val == 1:
            return "rgba(0,180,0,0.22)"        # verde
        elif val == 2:
            return "rgba(230,160,0,0.22)"      # naranja
        else:
            return "rgba(120,120,120,0.18)"    # gris para otros !=0

    for i, v in enumerate(stim_sync):
        if (v != 0) and (not in_block):
            in_block = True
            curr_val = int(v)
            start_i  = i
        elif in_block and ( (v == 0) or (int(v) != curr_val) or (i == len(stim_sync)-1) ):
            # fin de bloque en el índice anterior (o en este si es el último)
            end_i = i if (v == 0 or int(v) != curr_val) else i+1
            x0 = t_sync[start_i]
            x1 = t_sync[end_i-1] if end_i-1 < len(t_sync) else t_sync[-1]
            shapes.append(dict(
                type="rect",
                xref="x",
                yref="paper",    # cubre verticalmente toda la figura
                x0=x0, x1=x1,
                y0=0.0, y1=1.0,
                fillcolor=color_for(curr_val),
                line=dict(width=0),
                layer="below"
            ))
            in_block = False
            curr_val = 0

    # --- Figura interactiva (2 filas, x compartida) ---
    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=("EMG (ECR/FCR)", "Aceleración Z [g]")
    )

    # EMG en mV
    fig.add_trace(go.Scatter(
        x=t_sync, y=emg_ext_ds*1e3, mode="lines", name="EMG ECR [mV]"
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=t_sync, y=emg_flex_ds*1e3, mode="lines", name="EMG FCR [mV]"
    ), row=1, col=1)

    # Aceleración
    fig.add_trace(go.Scatter(
        x=t_sync, y=acc_sync, mode="lines", name="Aceleración Z [g]"
    ), row=2, col=1)

    # Franjas (sobre toda la figura con yref="paper")
    fig.update_layout(shapes=shapes)

    # Ejes y estética
    fig.update_yaxes(title_text="mV", row=1, col=1)
    fig.update_yaxes(title_text="g",  row=2, col=1)
    fig.update_xaxes(title_text="Tiempo (s)", row=2, col=1)

    fig.update_layout(
        title="EMG y Aceleración con franjas de estimulación",
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1.0),
        margin=dict(l=60, r=20, t=60, b=50)
    )

    # Guardar HTML
    out_html = f"emg_accel_stim_bands_{ACC_CSV.stem}.html"
    fig.write_html(out_html, include_plotlyjs="cdn")
    print(f"[OK] Gráfico interactivo guardado: {out_html}")

if __name__ == "__main__":
    main()
