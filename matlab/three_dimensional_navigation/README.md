# 3D UAV Visible-Light-Aware Navigation

Run:

```matlab
run('uav_vlc_3d_demo.m')
```

The first version compares:

- Baseline 3D A*: minimizes geometric path length.
- Communication-aware 3D A*: jointly considers VLC quality, obstacle
  clearance, vertical movement, and path length.

The simulation includes a `40 m x 30 m x 12 m` warehouse, racks with different
heights, 48 ceiling LEDs, line-of-sight blockage, a Lambertian VLC quality
field, 26-neighbor 3D A*, trajectory shortcutting, metrics, and automatic
figure export.

Results are saved to `results/three_dimensional_navigation/`.
