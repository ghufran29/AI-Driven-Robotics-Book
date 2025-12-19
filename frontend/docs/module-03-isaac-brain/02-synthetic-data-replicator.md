# Chapter 2: Infinite Data (Synthetic Generation)

## Using Replicator in Isaac Sim to Generate Labeled Training Data

This chapter covers the use of Isaac Sim Replicator for generating synthetic training data, including RGB images and segmentation masks for AI models. Synthetic data generation is a powerful approach that allows you to create large, diverse, and perfectly labeled datasets without the cost and time associated with real-world data collection.

### What is Isaac Sim Replicator?

Isaac Sim Replicator is a synthetic data generation framework built on NVIDIA Omniverse. It allows you to:

- Generate photorealistic RGB images with realistic lighting conditions
- Create precise segmentation masks with per-pixel labeling
- Apply domain randomization to increase dataset diversity
- Generate depth maps, normal maps, and other sensor data
- Create large datasets automatically without manual annotation

### Replicator Architecture

The Replicator framework follows this workflow:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   USD Scene     │    │   Replicator    │    │   Synthetic     │
│   with Assets   │───►│   Pipeline      │───►│   Dataset       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Domain         │    │  Annotation     │    │  RGB Images +   │
│  Randomization  │    │  Generation     │    │  Segmentation   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Installing and Setting Up Replicator

Replicator is included with Isaac Sim. To use it in your scripts:

```python
import omni.replicator.core as rep
```

### Basic Replicator Pipeline

Here's a simple example of how to set up a basic Replicator pipeline:

```python
import omni.replicator.core as rep

# Initialize Replicator
rep = rep.get_omniverse_replicator()

# Create a camera
camera = rep.create.camera(
    position=(-5, -5, 5),
    rotation=(0.7071, 0, 0, 0.7071)
)

# Register the camera
with rep.trigger.on_frame(num_frames=100):
    with rep.get.light():
        rep.modify.visibility(
            visibility=rep.random.uniform(0, 1),
            count=1
        )

    with rep.get.camera():
        rep.modify.pose(
            position=rep.distribution.uniform((-10, -10, 1), (10, 10, 10)),
            rotation=rep.distribution.uniform((-1, -1, -1, -1), (1, 1, 1, 1))
        )

# Define outputs
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="path/to/output", rgb=True, seg=True)
writer.attach([camera])
```

## Configuring Domain Randomization

Domain randomization is a technique used to increase the diversity of synthetic data by varying environmental parameters. This helps bridge the sim-to-real gap by making models more robust to variations they might encounter in the real world.

### Lighting Randomization

```python
# Randomize lighting conditions
with rep.get.light():
    # Random light positions
    rep.modify.pose(
        position=rep.distribution.uniform((-10, -10, 5), (10, 10, 15))
    )

    # Random light intensities
    rep.modify.intensity(
        intensity=rep.distribution.uniform(100, 10000)
    )

    # Random light colors
    rep.modify.color(
        color=rep.distribution.uniform((0.5, 0.5, 0.5), (1, 1, 1))
    )
```

### Material Randomization

```python
# Randomize materials for domain randomization
def randomize_materials(prims):
    with prims:
        # Random diffuse color
        rep.randomizer.material(
            diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
            metallic=rep.distribution.uniform(0, 1),
            roughness=rep.distribution.uniform(0, 1)
        )
    return prims

# Apply to all meshes in the scene
all_meshes = rep.get.mesh()
randomize_materials(all_meshes)
```

### Texture Randomization

```python
# Apply random textures from a library
def apply_random_textures(prims):
    with prims:
        rep.randomizer.texture(
            texture_path=["path/to/texture1", "path/to/texture2", "path/to/texture3"],
            scale=rep.distribution.uniform(0.1, 1.0)
        )
    return prims
```

### Camera Position Randomization

```python
# Randomize camera positions and orientations
with rep.get.camera():
    rep.modify.pose(
        position=rep.distribution.uniform((-5, -5, 1), (5, 5, 10)),
        rotation=rep.distribution.uniform((-0.5, -0.5, -0.5, -0.5), (0.5, 0.5, 0.5, 0.5))
    )
```

## Writing Replicator Scripts for RGB and Segmentation

### RGB Image Generation

```python
import omni.replicator.core as rep
import numpy as np

def setup_rgb_generation():
    # Create a writer for RGB images
    rgb_writer = rep.WriterRegistry.get("BasicWriter")
    rgb_writer.initialize(
        output_dir="output/rgb_images",
        rgb=True,
        camera_semantic_segmentation=False
    )

    # Get all cameras in the scene
    cameras = rep.get.camera()

    # Attach the writer to the cameras
    rgb_writer.attach(cameras)

    return rgb_writer

# Trigger the generation
with rep.trigger.on_frame(num_frames=1000):
    # Your randomization code here
    pass
```

### Segmentation Mask Generation

```python
def setup_segmentation_generation():
    # Create a writer for segmentation masks
    seg_writer = rep.WriterRegistry.get("BasicWriter")
    seg_writer.initialize(
        output_dir="output/segmentation_masks",
        rgb=False,
        camera_semantic_segmentation=True
    )

    # Create a semantic schema
    semantic_schema = rep.create.semantic_segmentation(
        prim=rep.get.mesh(),
        class_name="robot"
    )

    cameras = rep.get.camera()
    seg_writer.attach(cameras)

    return seg_writer, semantic_schema
```

### Combined RGB and Segmentation Pipeline

```python
import omni.replicator.core as rep

def create_combined_pipeline(output_dir, num_frames=1000):
    # Initialize the writer for both RGB and segmentation
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        camera_semantic_segmentation=True,
        depth=True,
        bbox_2d_tight=True,
        bbox_2d_loose=True
    )

    # Get cameras and attach writer
    cameras = rep.get.camera()
    writer.attach(cameras)

    # Set up triggers for data generation
    with rep.trigger.on_frame(num_frames=num_frames):
        # Randomize lighting
        with rep.get.light():
            rep.modify.intensity(
                intensity=rep.distribution.uniform(500, 5000)
            )
            rep.modify.pose(
                position=rep.distribution.uniform((-10, -10, 5), (10, 10, 15))
            )

        # Randomize camera positions
        with rep.get.camera():
            rep.modify.pose(
                position=rep.distribution.uniform((-3, -3, 1), (3, 3, 5)),
                rotation=rep.distribution.uniform((-0.3, -0.3, -0.3, -0.3), (0.3, 0.3, 0.3, 0.3))
            )

        # Randomize object materials
        with rep.get.mesh():
            rep.randomizer.material(
                diffuse=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)),
                metallic=rep.distribution.uniform(0, 0.5),
                roughness=rep.distribution.uniform(0.1, 1)
            )

    return writer
```

## Generating 100+ Synthetic Images with Labels

To generate 100+ synthetic images with proper labels, follow this approach:

### Step 1: Define Your Asset Library

```python
# Define paths to your assets
ASSET_PATHS = {
    "robots": ["path/to/robot1.usd", "path/to/robot2.usd"],
    "environments": ["path/to/env1.usd", "path/to/env2.usd"],
    "objects": ["path/to/obj1.usd", "path/to/obj2.usd", "path/to/obj3.usd"]
}
```

### Step 2: Create a Comprehensive Pipeline

```python
def generate_large_dataset(output_dir, target_count=100):
    # Initialize writer
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=output_dir,
        rgb=True,
        camera_semantic_segmentation=True,
        instance_segmentation=True,
        depth=True,
        bbox_2d_tight=True
    )

    # Get camera
    camera = rep.get.camera()
    writer.attach(camera)

    # Generate a large dataset with variations
    with rep.trigger.on_frame(num_frames=target_count):
        # Randomize multiple aspects simultaneously
        with rep.get.light():
            rep.modify.pose(
                position=rep.distribution.uniform((-10, -10, 5), (10, 10, 15)),
                intensity=rep.distribution.uniform(100, 10000)
            )

        # Randomize robot pose
        with rep.get.prim(path="/World/Robot"):
            rep.modify.pose(
                position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 0.5)),
                rotation=rep.distribution.uniform((-0.5, -0.5, -0.5, -0.5), (0.5, 0.5, 0.5, 0.5))
            )

        # Randomize background objects
        with rep.get.prim(path="/World/Obstacles/*"):
            rep.modify.pose(
                position=rep.distribution.uniform((-5, -5, 0.5), (5, 5, 0.5))
            )

    print(f"Generated {target_count} synthetic images with RGB and segmentation labels")
```

### Step 3: Run the Generation Process

```python
if __name__ == "__main__":
    # Generate 500 images as an example
    generate_large_dataset("output/synthetic_data", target_count=500)
```

## Quality Assurance for Synthetic Datasets

### Validation Script

```python
import os
import cv2
import numpy as np

def validate_synthetic_dataset(data_dir):
    """Validate the synthetic dataset for quality and completeness"""

    rgb_dir = os.path.join(data_dir, "rgb")
    seg_dir = os.path.join(data_dir, "seg")

    # Check that directories exist
    if not os.path.exists(rgb_dir) or not os.path.exists(seg_dir):
        print("ERROR: RGB or segmentation directories missing")
        return False

    # Get file lists
    rgb_files = [f for f in os.listdir(rgb_dir) if f.endswith('.png')]
    seg_files = [f for f in os.listdir(seg_dir) if f.endswith('.png')]

    # Check counts
    if len(rgb_files) != len(seg_files):
        print(f"ERROR: Mismatch in file counts - RGB: {len(rgb_files)}, Seg: {len(seg_files)}")
        return False

    # Validate each pair
    for rgb_file in rgb_files:
        seg_file = rgb_file.replace('rgb', 'seg')
        if seg_file not in seg_files:
            print(f"ERROR: Missing segmentation for {rgb_file}")
            return False

        # Load and validate images
        rgb_img = cv2.imread(os.path.join(rgb_dir, rgb_file))
        seg_img = cv2.imread(os.path.join(seg_dir, seg_file))

        if rgb_img is None or seg_img is None:
            print(f"ERROR: Could not load images for {rgb_file}")
            return False

        if rgb_img.shape[:2] != seg_img.shape[:2]:
            print(f"ERROR: Shape mismatch for {rgb_file}")
            return False

    print(f"SUCCESS: Validated {len(rgb_files)} image pairs successfully")
    return True

# Usage
validate_synthetic_dataset("output/synthetic_data")
```

## Performance Optimization Tips

### 1. Batch Processing
Process multiple frames simultaneously to improve throughput:

```python
# Process in larger batches
with rep.trigger.on_frame(num_frames=100, batch_time=0.1):
    # Your randomization here
```

### 2. Parallel Processing
Run multiple Replicator instances in parallel:

```python
# Create multiple writer instances for parallel generation
def create_parallel_pipeline(num_workers=4):
    pipelines = []
    for i in range(num_workers):
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=f"output/synthetic_data_worker_{i}",
            rgb=True,
            camera_semantic_segmentation=True
        )
        camera = rep.get.camera()
        writer.attach(camera)
        pipelines.append(writer)
    return pipelines
```

### 3. Memory Management
Monitor and optimize memory usage:

```python
# Clear cache periodically
rep.utils.clear_cache()
```

## Next Steps

This chapter has covered the fundamentals of synthetic data generation using Isaac Sim Replicator. In the next chapter, we'll dive into Visual SLAM (VSLAM) implementation using Isaac ROS GEMs, where we'll use the synthetic data you've generated to train and test perception systems.

The synthetic data generation capabilities you've learned will be crucial for creating robust perception systems that can handle various real-world scenarios, lighting conditions, and environmental variations.