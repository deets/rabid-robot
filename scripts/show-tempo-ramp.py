import sys
import pandas as pd
import plotly.graph_objects as go
import numpy as np


df = pd.read_csv(sys.argv[1])
fig = go.Figure()
fig.add_trace(
    go.Scatter(
        x=df.time,
        y=df.value,
        mode="lines",
    )
)
speed = np.diff(df.value.values)

fig.add_trace(
    go.Scatter(
        x=df.time.values[:-1],
        y=speed,
        mode="lines",
    )
)
fig.show()
