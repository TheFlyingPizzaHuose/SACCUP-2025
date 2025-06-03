import csv

def process_sensor_data(input_file):
    # Dictionary to hold sensor data lists
    sensor_data = {str(i): [] for i in range(7)}

    # Read and parse input file
    with open(input_file, 'r') as f:
        for line in f:
            parts = line.strip().split('|')
            if len(parts) < 2:
                continue  # skip invalid lines
            sensor_id = parts[1]
            if sensor_id in sensor_data:
                sensor_data[sensor_id].append(parts)

    # Write each sensor's data to its own CSV
    for sid, rows in sensor_data.items():
        if not rows:
            continue  # skip empty sets
        with open(f'sensor_{sid}.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in rows:
                writer.writerow(row)

if __name__ == "__main__":
    process_sensor_data("input.txt")