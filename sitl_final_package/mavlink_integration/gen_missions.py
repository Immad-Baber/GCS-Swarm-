import json, math

# Drone spawn location from start_sitl.sh
lat = 33.6844
lon = 73.0479
alt = 10.0

lat_m = 1.0 / 111320.0
lon_m = 1.0 / (111320.0 * math.cos(math.radians(lat)))

side = 150  # 150m side length

# Mission 2: Perfect Closed Square
# Start -> North -> North-East -> East -> back to Start
m2 = {
    "waypoints": [
        {"latitude": lat,                        "longitude": lon,                        "altitude": alt},
        {"latitude": lat + side * lat_m,         "longitude": lon,                        "altitude": alt},
        {"latitude": lat + side * lat_m,         "longitude": lon + side * lon_m,         "altitude": alt},
        {"latitude": lat,                        "longitude": lon + side * lon_m,         "altitude": alt},
        {"latitude": lat,                        "longitude": lon,                        "altitude": alt},
    ]
}
with open("mission2.json", "w") as f:
    json.dump(m2, f, indent=2)
print("mission2.json (Square) written with", len(m2["waypoints"]), "waypoints")

# Mission 3: Perfect Closed Equilateral Triangle
# Apex at top-center, base on bottom
h = side * math.sqrt(3) / 2
m3 = {
    "waypoints": [
        {"latitude": lat,              "longitude": lon,                          "altitude": alt},
        {"latitude": lat + h * lat_m,  "longitude": lon + (side / 2.0) * lon_m,   "altitude": alt},
        {"latitude": lat,              "longitude": lon + side * lon_m,            "altitude": alt},
        {"latitude": lat,              "longitude": lon,                          "altitude": alt},
    ]
}
with open("mission3.json", "w") as f:
    json.dump(m3, f, indent=2)
print("mission3.json (Triangle) written with", len(m3["waypoints"]), "waypoints")

print("\nDrone spawn:", lat, lon)
print("Side length:", side, "m | Triangle height:", round(h, 1), "m")
