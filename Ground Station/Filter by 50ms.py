import pandas as pd
import csv
from pyproj import Proj, Transformer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed even if unused directly


def convert_to_decimal_degrees(coord):
    coord = float(coord)
    if coord < 10000:  # Latitude: ddmm.mmmmmm
        degrees = int(coord / 100)
        minutes = coord - (degrees * 100)
    else:  # Longitude: dddmm.mmmmmm
        degrees = int(coord / 100)
        minutes = coord - (degrees * 100)
    return degrees + (minutes / 60)

# Set expected number of columns
EXPECTED_COLS = 4  # change this to match your CSV structure

valid_rows = []

# Open CSV and read line-by-line
with open('sensor_0.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if len(row) == EXPECTED_COLS:
            valid_rows.append(row)

# Convert to DataFrame
df_gps = pd.DataFrame(valid_rows, columns=['timestamp', 'col2', 'lat', 'lon'])  # update headers

# Make sure the timestamp column is in integer format
df_gps['timestamp'] = df_gps['timestamp'].astype(int)

# Round timestamps to the nearest 50 ms (50,000 µs)
df_gps['rounded_ts'] = (df_gps['timestamp'] / 50000).round() * 50000

# Drop duplicates so we keep only one row per 50ms bucket
df_filtered = df_gps.drop_duplicates(subset='rounded_ts')

# Optional: drop the helper column if you don't need it
df_filtered = df_filtered.drop(columns=['rounded_ts'])

# Step 3: Convert lat/lon columns (assumed col 3 and 4, 0-indexed as 2 and 3) to float
df_filtered.iloc[:, 2] = df_filtered.iloc[:, 2].astype(float).apply(convert_to_decimal_degrees)  # Latitude
df_filtered.iloc[:, 3] = df_filtered.iloc[:, 3].astype(float).apply(convert_to_decimal_degrees)  # Longitude

# Step 4: Set up projection system — UTM or local projection
start_lat = df_filtered.iloc[0, 2]
start_lon = df_filtered.iloc[0, 3]

# Create a transformer to convert from lat/lon to local meters
# Use a local Azimuthal Equidistant projection centered at the starting point
proj_str = f"+proj=aeqd +lat_0={start_lat} +lon_0={start_lon} +units=m +datum=WGS84"
proj = Proj(proj_str)
transformer = Transformer.from_proj("epsg:4326", proj, always_xy=True)

# Step 5: Convert all lat/lon to x/y in meters from the starting point
lons = df_filtered.iloc[:, 3].tolist()
lats = df_filtered.iloc[:, 2].tolist()

xs, ys = transformer.transform(lons, lats)

# Step 6: Store result in DataFrame
df_filtered['x_meters'] = xs
df_filtered['y_meters'] = ys

# Now df contains x/y positions in meters relative to the first point
print(df_filtered[['x_meters', 'y_meters']].head())

# Optional: save to file
df_filtered.to_csv("meter_pos.csv", index=False)

# Step 2: Read second file
df_alt = pd.read_csv('Starlance_Altitude_Data_Edited.csv', sep=',', header=None, names=['time', 'alt_ft', 'temp', 'other'])

# Step 3: Convert altitude from feet to meters
df_alt['alt_meters'] = df_alt['alt_ft'].astype(float) * 0.3048

print(df_alt.tail())

# Step 1: Trim both to the same number of rows, aligned by the end
min_len = min(len(df_filtered), len(df_alt))
df_gps_aligned = df_filtered.tail(min_len).reset_index(drop=True)
df_alt_aligned = df_alt.tail(min_len).reset_index(drop=True)

# Step 2: Combine the two
df_combined = pd.concat([df_gps_aligned, df_alt_aligned['alt_meters']], axis=1)

# Done
print(df_combined.head())
df_combined.to_csv("gps_with_aligned_altitude.csv", index=False)

# Set up figure with multiple 3D subplots
fig = plt.figure(figsize=(18, 6))

# Extract coordinates
x = df_combined['x_meters']
y = df_combined['y_meters']
z = df_combined['alt_meters']

# View 1: Isometric
ax1 = fig.add_subplot(131, projection='3d')
ax1.plot(x, y, z, color='blue')
ax1.set_title("Isometric View")
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Altitude (m)")
ax1.view_init(elev=30, azim=45)

# View 2: Top-down (XY plane)
ax2 = fig.add_subplot(132, projection='3d')
ax2.plot(x, y, z, color='green')
ax2.set_title("Top-Down View")
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_zlabel("Altitude (m)")
ax2.view_init(elev=90, azim=-90)

# View 3: Side view (XZ plane)
ax3 = fig.add_subplot(133, projection='3d')
ax3.plot(x, y, z, color='red')
ax3.set_title("Side View")
ax3.set_xlabel("X (m)")
ax3.set_ylabel("Y (m)")
ax3.set_zlabel("Altitude (m)")
ax3.view_init(elev=0, azim=0)

plt.tight_layout()
plt.show()