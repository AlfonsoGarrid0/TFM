# -*- coding: utf-8 -*-
"""
Pipeline de separación (artefacto vs actividad muscular) y análisis de fase
para ficheros .MAT del Quattrocento (p.ej. SUJETO4_QUA_ACC_STIMmed.mat).

Ramas de procesado (SEPARADAS):
- ARTEFACTO (alta frecuencia): HPF Butterworth 2º orden @ 500 Hz (filtfilt)
- ACTIVIDAD EMG (baja-media): Notches (red 50/100 Hz y armónicos de estim), luego BPF 10–90 Hz (2º, filtfilt)

Estimación de f_tremor:
- Desde la envolvente (Hilbert) del canal elegido (ECR/FCR), Welch en [3..12] Hz

Análisis de fase:
- Envolventes (artefacto y actividad) -> LP adaptativo -> BPF ±1 Hz en torno a f_tremor
- Fase vía Hilbert, “gating” por amplitud, máscara de estimulación, y centrado de referencia
"""

import numpy as np
from pathlib import Path
from scipy.io import loadmat
from scipy.signal import butter, filtfilt, iirnotch, welch, hilbert
import matplotlib.pyplot as plt

# =========================
# CONFIG
# =========================
MAT_PATH            = Path(r"SUJETO3_QUA_EMG_STIMmed.mat")
# Data: N x 3 = [ECR, FCR, AUX]; Time (N); SamplingFreq (opcional)

MAINS_FREQ          = 50.0          # España: 50 Hz
INCLUDE_MAINS       = True          # aplicar notch de red
STIM_F0             = 100.0         # Hz base de estimulación (armónicos 100..900 Hz)
N_HARMONICS         = 9             # 100..900 Hz
Q_MAINS             = 30.0          # Q del notch de red (ancho moderado)
Q_STIM              = 50.0          # Q notches armónicos de estimulación (estrechos)
BPF_ACT_LOW         = 10.0          # BPF para ACTIVIDAD EMG (baja-media)
BPF_ACT_HIGH        = 90.0
HPF_ARTEFACT        = 500.0         # HPF para ARTEFACTO (alta)
TREMOR_SEARCH       = (3.0, 12.0)   # banda de búsqueda de temblor (Hz)
TREMOR_BW           = 1.0           # ±1 Hz para BPF estrecho (fase)
PSD_NPERSEG         = 4096
POLAR_BINS_DEG      = 20
TREMOR_FROM         = "FCR"         # "ECR" o "FCR" para estimar f_tremor
SHOW_PHASE_SIGNALS  = True
PLOT_INTERMEDIATE   = True
GATE_PERCENTILE     = 70.0          # umbral (percentil) de amplitud para enmascarar fases ruidosas
CENTER_REFERENCE    = True          # centrar artefacto para que Act(ECR) vs Art(ECR) -> 0°
# Estimulación: cómo construir la máscara
#   "any"    -> aux > 0
#   "radial" -> aux == 1  (ECR)
#   "median" -> aux == 2  (FCR)
STIM_MASK_MODE      = "any"

# =========================
# UTILIDADES
# =========================
def _unwrap(x):
    x = np.array(x).squeeze()
    while isinstance(x, np.ndarray) and x.dtype == object:
        x = x.item()
        x = np.array(x).squeeze()
    return np.array(x)

def load_quattroccento_mat(path: Path):
    mat  = loadmat(path, squeeze_me=True, struct_as_record=False)
    data = _unwrap(mat["Data"])          # N x 3  (ECR, FCR, AUX)
    time = _unwrap(mat["Time"]).ravel()  # N
    if "SamplingFreq" in mat:
        fs  = float(np.array(mat["SamplingFreq"]).squeeze())
    else:
        fs  = 1.0 / np.median(np.diff(time))
    if data.ndim != 2 or data.shape[1] < 3:
        raise ValueError(f"Data debería ser N x 3 (ECR,FCR,AUX). shape={data.shape}")
    ecr = data[:, 0].astype(float)
    fcr = data[:, 1].astype(float)
    aux = data[:, 2].astype(float)
    return time, ecr, fcr, aux, fs

def butter_sos_bpf(low, high, fs, order=2):
    nyq = 0.5 * fs
    lowc = max(1e-6, low / nyq)
    highc = min(0.999999, high / nyq)
    b, a = butter(order, [lowc, highc], btype='band')
    return b, a

def butter_sos_hpf(fc, fs, order=2):
    nyq = 0.5 * fs
    wc = min(0.999999, fc / nyq)
    b, a = butter(order, wc, btype='high')
    return b, a

def apply_notch(x, fs, f0, Q):
    w0 = f0 / (fs/2.0)
    if w0 >= 1.0:
        return x
    b, a = iirnotch(w0, Q)
    return filtfilt(b, a, x)  # fase cero

def chain_notches(x, fs, mains_freq, include_mains, stim_f0, n_harmonics, q_mains, q_stim):
    y = x.copy()
    if include_mains and mains_freq > 0:
        y = apply_notch(y, fs, mains_freq, q_mains)
        if 2*mains_freq < fs/2:
            y = apply_notch(y, fs, 2*mains_freq, q_mains)
    for k in range(1, n_harmonics+1):
        fk = k * stim_f0
        if fk >= fs/2:
            break
        y = apply_notch(y, fs, fk, q_stim)
    return y

def estimate_tremor_peak(f, Pxx, search_band):
    fmin, fmax = search_band
    mask = (f >= fmin) & (f <= fmax)
    if not np.any(mask):
        return f[np.argmax(Pxx)]
    idx = np.argmax(Pxx[mask])
    return f[mask][idx]

def narrowband_filter(x, fs, f0, bw):
    low = max(0.1, f0 - bw)
    high = f0 + bw
    b, a = butter_sos_bpf(low, high, fs, order=2)
    return filtfilt(b, a, x)

def lp_filter(x, fs, fc=20.0, order=2):
    b, a = butter(order, fc/(fs/2), btype='low')
    return filtfilt(b, a, x)

def circ_R(theta):
    z = np.exp(1j*theta)
    return np.abs(np.mean(z))

def norm99(x):
    s = np.percentile(np.abs(x), 99.5) + 1e-12
    return x / s

def phase_and_mask(x_nb, perc=70.0):
    """
    Devuelve fase y máscara basada en amplitud instantánea
    (umbral = percentil 'perc' de |hilbert(x_nb)|).
    """
    analytic = hilbert(x_nb)
    amp = np.abs(analytic)
    thr = np.percentile(amp, perc)
    mask = amp >= thr
    phi = np.angle(analytic)
    return phi, mask

def build_stim_mask(aux, mode="any"):
    aux = np.asarray(aux).astype(float)
    if mode == "any":
        return aux > 0
    if mode == "radial":
        return aux == 1
    if mode == "median":
        return aux == 2
    return np.ones_like(aux, dtype=bool)

# =========================
# PIPELINE
# =========================
def main():
    # --- Cargar ---
    t, ecr_raw, fcr_raw, aux, fs = load_quattroccento_mat(MAT_PATH)
    print(f"Fs = {fs:.2f} Hz | N = {len(t)}")

    # ============================================================
    # RUTA 1: ARTEFACTO — HPF (sin notches ni BPF 10–90)
    # ============================================================
    b_hpf, a_hpf = butter_sos_hpf(HPF_ARTEFACT, fs, order=2)
    artefact_ecr = filtfilt(b_hpf, a_hpf, ecr_raw)
    artefact_fcr = filtfilt(b_hpf, a_hpf, fcr_raw)

    # ============================================================
    # RUTA 2: ACTIVIDAD — Notches + BPF 10–90
    # ============================================================
    ecr_notched = chain_notches(ecr_raw, fs, MAINS_FREQ, INCLUDE_MAINS,
                                STIM_F0, N_HARMONICS, Q_MAINS, Q_STIM)
    fcr_notched = chain_notches(fcr_raw, fs, MAINS_FREQ, INCLUDE_MAINS,
                                STIM_F0, N_HARMONICS, Q_MAINS, Q_STIM)

    b_bpf, a_bpf = butter_sos_bpf(BPF_ACT_LOW, BPF_ACT_HIGH, fs, order=2)
    activity_ecr = filtfilt(b_bpf, a_bpf, ecr_notched)
    activity_fcr = filtfilt(b_bpf, a_bpf, fcr_notched)

    # --- Estimar f_tremor desde la envolvente del canal elegido ---
    ref = activity_fcr if TREMOR_FROM.upper() == "FCR" else activity_ecr
    env_ref = np.abs(hilbert(ref))
    env_ref_lp = lp_filter(env_ref, fs, fc=20.0)
    f, Pxx = welch(env_ref_lp, fs=fs, nperseg=min(PSD_NPERSEG, len(env_ref_lp)//2))
    f_tremor = estimate_tremor_peak(f, Pxx, TREMOR_SEARCH)
    print(f"f_tremor (desde envolvente {TREMOR_FROM.upper()}) = {f_tremor:.3f} Hz")

    # --- LP para envolventes (adaptativo a f_tremor) ---
    ENV_LP_FC = max(2.0, min(40.0, 2.0*f_tremor))

    # Envolventes con AMPLITUD ANALÍTICA (artefacto y actividad)  ***clave***
    artefact_ecr_env = lp_filter(np.abs(hilbert(artefact_ecr)), fs, fc=ENV_LP_FC)
    artefact_fcr_env = lp_filter(np.abs(hilbert(artefact_fcr)), fs, fc=ENV_LP_FC)
    activity_ecr_env = lp_filter(np.abs(hilbert(activity_ecr)), fs, fc=ENV_LP_FC)
    activity_fcr_env = lp_filter(np.abs(hilbert(activity_fcr)), fs, fc=ENV_LP_FC)

    # --- BPF estrecho ±1 Hz (SOLO para fase) sobre ENVOLVENTES ---
    artefact_ecr_nb = narrowband_filter(artefact_ecr_env, fs, f_tremor, TREMOR_BW)
    artefact_fcr_nb = narrowband_filter(artefact_fcr_env, fs, f_tremor, TREMOR_BW)
    activity_ecr_nb = narrowband_filter(activity_ecr_env, fs, f_tremor, TREMOR_BW)
    activity_fcr_nb = narrowband_filter(activity_fcr_env, fs, f_tremor, TREMOR_BW)

    # --- Fase instantánea + máscaras por amplitud (“gating”) ---
    phi_art_ecr, m_art_ecr = phase_and_mask(artefact_ecr_nb, perc=GATE_PERCENTILE)
    phi_art_fcr, m_art_fcr = phase_and_mask(artefact_fcr_nb, perc=GATE_PERCENTILE)
    phi_act_ecr, m_act_ecr = phase_and_mask(activity_ecr_nb, perc=GATE_PERCENTILE)
    phi_act_fcr, m_act_fcr = phase_and_mask(activity_fcr_nb, perc=GATE_PERCENTILE)

    # --- Máscara de estimulación (según AUX) ---
    stim_mask = build_stim_mask(aux, STIM_MASK_MODE)
    stim_mask = stim_mask.astype(bool)

    # Intersecciones de máscaras para comparar al MISMO tiempo
    m_ecr      = m_art_ecr & m_act_ecr & stim_mask
    m_fcr      = m_art_fcr & m_act_fcr & stim_mask
    m_ecr_fcr  = m_act_ecr & m_act_fcr & stim_mask

    # --- (opcional) Centrar referencia: Act(ECR) vs Art(ECR) -> 0° ---
    if CENTER_REFERENCE and np.any(m_ecr):
        offset = np.angle(np.mean(np.exp(1j*(phi_act_ecr[m_ecr] - phi_art_ecr[m_ecr]))))
        phi_art_ecr = phi_art_ecr + 0.0 - offset
        phi_art_fcr = phi_art_fcr + 0.0 - offset  # mismo giro para mantener coherencia
        # No se toca la actividad (solo referencia)

    # --- Diferencias de fase [0, 2π) SOLO en muestras válidas ---
    dphi_ecr      = np.mod(phi_act_ecr[m_ecr]     - phi_art_ecr[m_ecr],     2*np.pi)
    dphi_fcr      = np.mod(phi_act_fcr[m_fcr]     - phi_art_fcr[m_fcr],     2*np.pi)
    dphi_ecr_fcr  = np.mod(phi_act_ecr[m_ecr_fcr] - phi_act_fcr[m_ecr_fcr], 2*np.pi)

    # Medida de concentración circular
    print("R_ecr=", circ_R(dphi_ecr), "R_fcr=", circ_R(dphi_fcr), "R_ecr_fcr=", circ_R(dphi_ecr_fcr))

    # =========================
    # FIGURA 1 — EMG 10–90 vs Artefacto HPF 500 (ejes dobles)
    # =========================
    dur_show = t[-1]
    n_show = int(min(dur_show*fs, len(t)))
    ts = t[:n_show] - t[0]

    c_act = 'tab:blue'
    c_art = 'tab:orange'

    fig, (axE, axF) = plt.subplots(2, 1, figsize=(11, 5), sharex=True)

    # ECR
    axE.plot(ts, activity_ecr[:n_show], color=c_act,
             label='ECR — EMG limpia (10–90 Hz)')
    axE.set_ylabel('EMG')
    axE.legend(loc='upper left')
    axE.set_title('ECR: EMG limpia vs Artefacto (10 s)')
    axE.grid(True, alpha=0.25)
    axE2 = axE.twinx()
    axE2.plot(ts, artefact_ecr[:n_show], color=c_art, linestyle='--',
              label='ECR — Artefacto (HPF 500 Hz)', alpha=0.85)
    axE2.set_ylabel('Artefacto')
    l1, lb1 = axE.get_legend_handles_labels()
    l2, lb2 = axE2.get_legend_handles_labels()
    axE.legend(l1+l2, lb1+lb2, loc='upper right')

    # FCR
    axF.plot(ts, activity_fcr[:n_show], color=c_act,
             label='FCR — EMG limpia (10–90 Hz)')
    axF.set_xlabel('Tiempo (s)')
    axF.set_ylabel('EMG')
    axF.legend(loc='upper left')
    axF.set_title('FCR: EMG limpia vs Artefacto (10 s)')
    axF.grid(True, alpha=0.25)
    axF2 = axF.twinx()
    axF2.plot(ts, artefact_fcr[:n_show], color=c_art, linestyle='--',
              label='FCR — Artefacto (HPF 500 Hz)', alpha=0.85)
    axF2.set_ylabel('Artefacto')
    l1, lb1 = axF.get_legend_handles_labels()
    l2, lb2 = axF2.get_legend_handles_labels()
    axF.legend(l1+l2, lb1+lb2, loc='upper right')

    plt.tight_layout()
    plt.show()

    # =========================
    # FIGURA 2 — SEÑALES USADAS PARA FASE (ENVOLVENTES estrechas ±1 Hz)
    # =========================
    if SHOW_PHASE_SIGNALS:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 5), sharex=True)

        # ECR
        ax1.plot(ts, norm99(activity_ecr_nb[:n_show]), color=c_act,
                 label='ECR — Actividad (ENV BPF ±1 Hz)')
        ax1.set_ylabel('a.u.')
        ax1.set_title('Señales usadas para la fase — ECR')
        ax1.grid(True, alpha=0.3)
        ax1b = ax1.twinx()
        ax1b.plot(ts, norm99(artefact_ecr_nb[:n_show]), color=c_art, linestyle='--',
                  label='ECR — Artefacto (ENV BPF ±1 Hz)')
        ax1b.set_ylabel('a.u.')
        l1, lb1 = ax1.get_legend_handles_labels()
        l2, lb2 = ax1b.get_legend_handles_labels()
        ax1.legend(l1+l2, lb1+lb2, loc='upper right')

        # FCR
        ax2.plot(ts, norm99(activity_fcr_nb[:n_show]), color=c_act,
                 label='FCR — Actividad (ENV BPF ±1 Hz)')
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('a.u.')
        ax2.set_title('Señales usadas para la fase — FCR')
        ax2.grid(True, alpha=0.3)
        ax2b = ax2.twinx()
        ax2b.plot(ts, norm99(artefact_fcr_nb[:n_show]), color=c_art, linestyle='--',
                  label='FCR — Artefacto (ENV BPF ±1 Hz)')
        ax2b.set_ylabel('a.u.')
        l1, lb1 = ax2.get_legend_handles_labels()
        l2, lb2 = ax2b.get_legend_handles_labels()
        ax2.legend(l1+l2, lb1+lb2, loc='upper right')

        plt.tight_layout()
        plt.show()

    # =========================
    # PLOTS INTERMEDIOS (PSD + polares)
    # =========================
    if PLOT_INTERMEDIATE:
        # PSD de la envolvente (pico de temblor)
        plt.figure(figsize=(7,4))
        plt.semilogy(f, Pxx)
        plt.axvline(f_tremor, linestyle='--')
        plt.xlim(0, 30)
        plt.xlabel('Frecuencia (Hz)')
        plt.ylabel('PSD (a.u.)')
        plt.title(f'PSD envolvente {TREMOR_FROM.upper()} (pico en {f_tremor:.2f} Hz)')
        plt.tight_layout()

        # Histogramas polares (fase)
        def circ_hist_local(theta, bins_deg=20):
            if len(theta) == 0:
                return np.array([0]), np.array([0.0])
            bins = int(np.round(360.0 / bins_deg))
            edges = np.linspace(0, 2*np.pi, bins+1)
            theta = np.mod(theta, 2*np.pi)
            H, _ = np.histogram(theta, bins=edges)
            centers = (edges[:-1] + edges[1:]) / 2.0
            return H, centers

        H1, C1 = circ_hist_local(dphi_ecr,     POLAR_BINS_DEG)
        H2, C2 = circ_hist_local(dphi_fcr,     POLAR_BINS_DEG)
        H3, C3 = circ_hist_local(dphi_ecr_fcr, POLAR_BINS_DEG)

        fig = plt.figure(figsize=(15,4.8))
        ax1 = plt.subplot(1,3,1, projection='polar')
        ax2 = plt.subplot(1,3,2, projection='polar')
        ax3 = plt.subplot(1,3,3, projection='polar')

        def plot_polar_hist(ax, H, centers, title):
            theta = np.r_[centers, centers[0]]
            rho   = np.r_[H,       H[0]]
            ax.plot(theta, rho)
            ax.fill(theta, rho, alpha=0.3)
            ax.set_title(title, va='bottom')

        plot_polar_hist(ax1, H1, C1, 'Δϕ: Actividad ECR vs Artefacto')
        plot_polar_hist(ax2, H2, C2, 'Δϕ: Actividad FCR vs Artefacto')
        plot_polar_hist(ax3, H3, C3, 'Δϕ: Actividad ECR vs Actividad FCR')

        plt.suptitle(f'Hist. polares (bins {POLAR_BINS_DEG}°) — f_tremor={f_tremor:.2f} Hz')
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    main()
