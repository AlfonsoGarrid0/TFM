import numpy as np
import plotly.graph_objects as go

# 1) Carga los datos
data = np.loadtxt('test06_09_5.csv', delimiter=',', skiprows=1)
time       = data[:,0]
z_accel    = data[:,1]
stim_flag  = data[:,2].astype(int)

# 2) Prepara las franjas de estimulación
shapes = []
in_block = False
for i, flag in enumerate(stim_flag):
    if flag != 0 and not in_block:
        start_t = time[i]
        curr    = flag
        in_block = True
    elif in_block and (i == len(stim_flag)-1 or stim_flag[i] != curr):
        end_t = time[i]
        color = 'rgba(0,255,0,0.3)' if curr==1 else 'rgba(230,160,0,0.3)'
        shapes.append(dict(type="rect",
                           x0=start_t, x1=end_t,
                           y0=min(z_accel), y1=max(z_accel),
                           fillcolor=color, line_width=0))
        in_block = False

# 3) Crea la figura
fig = go.Figure()
fig.add_trace(go.Scatter(x=time, y=z_accel, mode='lines', name='z_accel'))

# 4) Añade las franjas
fig.update_layout(shapes=shapes,
                  xaxis_title='Tiempo (s)',
                  yaxis_title='Aceleración Z (g)',
                  title='Aceleración Z con Estimulación',
                  hovermode='x unified')

# 5) Guarda como HTML
fig.write_html('accel_with_stim_test06_09_5.html', include_plotlyjs='cdn')
print("Gráfico interactivo html guardado correctamente. Abrir en navegador.")
