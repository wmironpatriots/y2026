# Camera Transform3d Visualizer

This is a **portable, non-web** visualizer that renders camera positions from
WPILib `Transform3d` values using a local Python script.

## What it does

- Reads a JSON file of camera poses.
- Draws each camera as a coordinate frame in 3D.
- Shows an arrow for the camera look direction (forward +X).
- Shows the robot origin axes for reference.
- Can render to a window or save a PNG file.

## Input format

The default input file is `cameras.json` in this folder.

```json
{
	"cameras": [
		{
			"name": "FrontCam",
			"transform3d": {
				"x": 0.35,
				"y": 0.18,
				"z": 0.62,
				"roll": 0.0,
				"pitch": -15.0,
				"yaw": 0.0,
				"degrees": true
			}
		}
	]
}
```

### Rotation convention

Rotations use roll (X), pitch (Y), yaw (Z). They are applied in the WPILib
order: **roll → pitch → yaw** (equivalent to $R_z R_y R_x$).

## Usage

Install requirements and run the script:

```bash
python3 -m pip install -r requirements.txt
python3 visualize.py --input cameras.json
```

To save an image in headless mode:

```bash
python3 visualize.py --input cameras.json --output camera_poses.png
```

## CLI options

- `--input`: Path to the JSON file.
- `--output`: Save to a PNG (skips GUI).
- `--axes-length`: Axis length in meters.
- `--no-robot`: Hide the robot origin axes.
