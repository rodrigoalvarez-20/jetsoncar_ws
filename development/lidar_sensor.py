from sweeppy import Sweep
import numpy as np
import plotly.graph_objects as go
from time import sleep

fig = go.Figure()

# Create initial 3D scatter plot
fig.add_trace(go.Scatter3d(
    x=[], y=[],
    mode='markers',
    marker=dict(size=5, color=[], colorscale='Viridis', opacity=0.8)
))

# Set up layout
fig.update_layout(
    title="Real-time 3D LIDAR Visualization with Signal Strength",
    scene=dict(
        xaxis_title='X (meters)',
        yaxis_title='Y (meters)'
    )
)

#

with Sweep('/dev/ttyUSB0') as sweep:
    #print(sweep.get_motor_speed())
    #print(sweep.get_sample_rate())
    sweep.set_motor_speed(3)
    sweep.start_scanning()
    for scan in sweep.get_scans():
        print(scan.samples)
        #sampled_data = np.array([[x[0], x[1], x[2]] for x in scan.samples])
        #rescaled_
        #x = sampled_data[:,1] * np.cos(sampled_data[:,0])
        #y = sampled_data[:,1] * np.sin(sampled_data[:,0])
        #strengths = sampled_data[:,-1]
        #fig.data[0].update(
        #    x=x, y=y,
        #    marker=dict(color=strengths, colorscale='Viridis')
        #)