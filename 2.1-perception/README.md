# 2.1 Perception

> **Status: outline, not yet written.** To be built from the perception notebook. Follows [TRACK_TEMPLATE.md](../TRACK_TEMPLATE.md).

**The question:** given pictures from the car's camera, where are the cones and what colour are they?

**Where it sits:** the yellow boxes in the [system diagram](../system-architecture-diagram.png) — specifically the **camera pipeline**. We do not use lidar, so the lidar pipeline and the fusion of the two do not apply to our car. Input: stereo camera images. Output: cone positions and colours for SLAM and planning.

**Needs:** nothing from Modules 0 or 1. This track runs offline in a Jupyter notebook; the simulator does not render camera images, so there is no sim step.

## Planned shape
- **Hands-on:** the Jupyter notebook (data loading, detecting cones, measuring how good the detector is).
- **Check:** a score (for example detection accuracy) computed by the notebook.
- **See it work:** results drawn over the images.
- **Optional:** publish detections in the same message format the simulator uses for its emulated camera (`/camera/cones`, `eufs_msgs/ConeWithColorProbabilityArray`) so they could feed the rest of the stack.
- **What the real work is like:** to be written with the perception team (labelling data, training time, GPU needs).

## To decide
- Which dataset the notebook uses and whether it needs a GPU (decides if it runs in the dev container or on Colab).
- Whether to include the optional ROS step this term.
