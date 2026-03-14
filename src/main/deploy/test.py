import json

def generate_java_map(file_path, map_name):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    samples = []
    # Access the outer distance map
    dist_map = data.get("map", {})
    
    # Sort distances numerically to keep the Java map sequential
    sorted_distances = sorted(dist_map.keys(), key=float)
    
    for dist_str in sorted_distances:
        velocity_map = dist_map[dist_str].get("map", {})
        if not velocity_map:
            continue
            
        # Find the entry with the minimum velocity
        min_velocity_str = min(velocity_map.keys(), key=float)
        
        details = velocity_map[min_velocity_str]
        pitch = details.get("pitchRad")
        tof = details.get("timeOfFlightSeconds")
        velocity = float(min_velocity_str)
        
        samples.append(f"    {map_name}.addSample({dist_str}, new ProjectileParameters({pitch}, {velocity}, {tof}));")
    
    return samples

# Process both files
hub_lines = generate_java_map('HubShotMap.json', 'kHubShotMap')
ground_lines = generate_java_map('GroundShotMap.json', 'kGroundShotMap')

# Print the final Java block
print("public static final InterpolatingProjectileParametersTree kHubShotMap =")
print("      new InterpolatingProjectileParametersTree();")
print("")
print("public static final InterpolatingProjectileParametersTree kGroundShotMap =")
print("      new InterpolatingProjectileParametersTree();")
print("")
print("static {")
print("    // Hub Shot Map (Least Velocity Points)")
for line in hub_lines:
    print(line)
print("")
print("    // Ground Shot Map (Least Velocity Points)")
for line in ground_lines:
    print(line)
print("}")