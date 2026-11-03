# %% [markdown]
# # Thrust stand — VS Code native plotting
# Run cells with Shift+Enter. Plots appear in VS Code's Python Interactive
# window and can be opened in the built-in Plot Viewer.

# %%
from realtime_logger import (
    DEFAULT_TCP_PORT,
    DEFAULT_WIFI_HOST,
    ThrustStandSession,
)
from vscode_plotting import render_dashboard, save_recorded_plot, show_live

# Edit these values before running this cell.
CONNECTION = "offline"  # "serial", "wifi", or "offline"
SERIAL_PORT = "COM7"
WIFI_HOST = DEFAULT_WIFI_HOST
WINDOW_SECONDS = 30.0

try:
    session.close()
except NameError:
    pass

session = ThrustStandSession(window=WINDOW_SECONDS)
if CONNECTION == "serial":
    session.connect_serial(SERIAL_PORT)
elif CONNECTION == "wifi":
    session.connect_wifi(WIFI_HOST, DEFAULT_TCP_PORT)
elif CONNECTION != "offline":
    raise ValueError("CONNECTION must be serial, wifi, or offline")

print(f"CSV recording: {session.output}")
print("Connected:", session.sources[0].endpoint if session.sources else "offline")

# %% [markdown]
# ## Live plot
# The acquisition threads and CSV writer continue at full rate. This cell
# refreshes only the inline figure. Re-run it for another interval.

# %%
show_live(session, seconds=30, refresh_seconds=0.5)

# %% [markdown]
# ## Calibration commands
# Remove the leading comment only from the command you intend to send.

# %%
# session.tare()
# session.calibrate_cell(cell=1, force_lbf=2.20462)
# session.calibrate_cell(cell=2, force_lbf=2.20462)
# session.calibrate_cell(cell=3, force_lbf=2.20462)
# session.calibrate_cell(cell=4, force_lbf=2.20462)
# session.invert()
# session.reset_load_calibration()
# session.zero_current()
# session.calibrate_current(amps=20.0)
# session.zero_voltage()
# session.calibrate_voltage(volts=24.0)
# session.power_info()
# session.reset_power_calibration()

# %% [markdown]
# Run this after a calibration command to read the ESP response.

# %%
session.poll()
print("\\n".join(list(session.status_messages)[-10:]) or "No status messages yet.")

# %% [markdown]
# ## Record a complete test interval
# Run this immediately before the test.

# %%
session.start_recording()
print("Interval recording started; full-rate CSV recording is also active.")

# %% [markdown]
# Run this after the test. It preserves the entire marked interval in the PNG.

# %%
saved_plot = save_recorded_plot(session)
print(f"Saved plot: {saved_plot}")
render_dashboard(session, recorded=True)

# %% [markdown]
# ## Close cleanly
# Always run this before disconnecting the ESP or closing VS Code.

# %%
session.close()
print(f"Saved {session.rows_written} samples to {session.output}")
