import json
import math
import argparse
import os
import glob

# Standard 2024 Crescendo Field Width in meters. 
# Update this if you are using a different field year.
DEFAULT_FIELD_WIDTH = 8.21055

def mirror_heading(heading):
    """Negates the heading and normalizes it to [-pi, pi)."""
    new_heading = -heading
    return math.atan2(math.sin(new_heading), math.cos(new_heading))

def mirror_trajectory(input_path, output_path, field_width):
    try:
        with open(input_path, 'r') as f:
            traj = json.load(f)
    except Exception as e:
        print(f"Error reading {input_path}: {e}")
        return

    if "samples" not in traj:
        print(f"Skipping {input_path}: Does not appear to be a valid Choreo .traj file.")
        return

    for sample in traj["samples"]:
        # 1. Mirror Y position
        sample["y"] = field_width - sample["y"]

        # 2. Mirror Y velocity
        if "velocityY" in sample:
            sample["velocityY"] = -sample["velocityY"]

        # 3. Mirror Heading
        if "heading" in sample:
            sample["heading"] = mirror_heading(sample["heading"])

        # 4. Mirror Angular Velocity
        if "angularVelocity" in sample:
            sample["angularVelocity"] = -sample["angularVelocity"]

    try:
        with open(output_path, 'w') as f:
            json.dump(traj, f, indent=2)
        print(f"Successfully mirrored: {os.path.basename(output_path)}")
    except Exception as e:
        print(f"Error writing {output_path}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Mirror Choreo .traj files Left-to-Right for a single alliance.")
    parser.add_argument("input", help="Input .traj file or directory containing .traj files")
    parser.add_argument("-o", "--output", help="Output file or directory (defaults to appending '_mirror' to filename)", default=None)
    parser.add_argument("-w", "--width", type=float, default=DEFAULT_FIELD_WIDTH, help=f"Field width in meters (default: {DEFAULT_FIELD_WIDTH})")
    
    args = parser.parse_args()

    # Handle single file processing
    if os.path.isfile(args.input):
        if not args.input.endswith('.traj'):
            print("Warning: Input file does not have a .traj extension.")
        
        out_file = args.output
        if not out_file:
            base, ext = os.path.splitext(args.input)
            out_file = f"{base}_mirrored{ext}"
            
        mirror_trajectory(args.input, out_file, args.width)

    # Handle directory processing
    elif os.path.isdir(args.input):
        out_dir = args.output if args.output else args.input
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        traj_files = glob.glob(os.path.join(args.input, "*.traj"))
        if not traj_files:
            print(f"No .traj files found in directory: {args.input}")
            return

        for file_path in traj_files:
            # Skip files that were already mirrored to prevent double-mirroring
            if "_mirrored" in file_path:
                continue
                
            base_name = os.path.basename(file_path)
            name, ext = os.path.splitext(base_name)
            out_file = os.path.join(out_dir, f"{name}_mirrored{ext}")
            
            mirror_trajectory(file_path, out_file, args.width)
    else:
        print(f"Error: Input '{args.input}' is neither a valid file nor a directory.")

if __name__ == "__main__":
    main()