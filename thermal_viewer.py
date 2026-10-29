import serial
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
PORT = '/dev/cu.usbserial-0001'
BAUD = 115200
TEMP_THRESHOLD = 29.0  # Minimum temperature (°C) to qualify as a hotspot
HOTSPOT_RADIUS = 3     # Must match Arduino's radius setting (3 = 7x7 window)
# ---------------------

print(f"Connecting to {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# Setup the plot window
fig, ax = plt.subplots(figsize=(9, 7))
initial_grid = np.zeros((24, 32))

# Render heat map with smooth interpolation
cax = ax.imshow(initial_grid, cmap='inferno', vmin=20, vmax=35, interpolation='bicubic')
cbar = fig.colorbar(cax, label='Temperature (°C)')

# Scatter plot overlay for detected hotspots (cyan X markers)
hotspot_scatter = ax.scatter([], [], color='cyan', marker='x', s=120, linewidths=2.5, label='Hotspots')

ax.set_title("MLX90640 Real-Time Thermal Heat Map & Hotspot Counter")
ax.set_xlabel("X (Columns)")
ax.set_ylabel("Y (Rows)")
ax.legend(loc='upper right')

plt.ion()
plt.show()

def find_hotspots(grid, threshold, radius):
    """Scans the 32x24 grid for local maximums matching the radius and threshold criteria."""
    hotspots = []
    rows, cols = grid.shape
    for r in range(radius, rows - radius):
        for c in range(radius, cols - radius):
            val = grid[r, c]
            if val < threshold:
                continue
            
            # Check if strictly the highest pixel within the radius window
            is_local_max = True
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if dr == 0 and dc == 0:
                        continue
                    if grid[r + dr, c + dc] >= val:
                        is_local_max = False
                        break
                if not is_local_max:
                    break
            
            if is_local_max:
                hotspots.append((r, c, val)) # stores (row, col, temp)
    return hotspots

print("Listening for frames... Press Ctrl+C in terminal to stop.")

try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Look for the CSV data block header from Arduino
        if "--- FULL 32x24 FRAME CSV DATA ---" in line:
            rows = []
            while len(rows) < 24:
                row_line = ser.readline().decode('utf-8', errors='ignore').strip()
                if row_line and not "===" in row_line and not "---" in row_line:
                    try:
                        vals = [float(x) for x in row_line.split(',')]
                        if len(vals) == 32:
                            rows.append(vals)
                    except ValueError:
                        pass
            
            if len(rows) == 24:
                data_grid = np.array(rows)
                
                # Run Python-side hotspot detection
                hotspots = find_hotspots(data_grid, TEMP_THRESHOLD, HOTSPOT_RADIUS)
                count = len(hotspots)
                
                # Update heat map image data
                cax.set_data(data_grid)
                cax.set_clim(vmin=np.min(data_grid), vmax=np.max(data_grid))
                
                # Update hotspot overlay coordinates (Matplotlib expects [X, Y] which is [col, row])
                if count > 0:
                    xs = [h[1] for h in hotspots]
                    ys = [h[0] for h in hotspots]
                    hotspot_scatter.set_offsets(np.c_[xs, ys])
                else:
                    hotspot_scatter.set_offsets(np.empty((0, 2)))
                
                # Update dynamic title with live count
                ax.set_title(f"MLX90640 Thermal Heat Map | Hotspots Detected: {count}")
                
                # Refresh the plot window
                plt.draw()
                plt.pause(0.001)

except KeyboardInterrupt:
    ser.close()
    print("\nSerial connection closed. Exiting.")
