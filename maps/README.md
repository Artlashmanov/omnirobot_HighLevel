# Saved robot maps

Runtime map captures are stored here on the robot. Generated map folders are ignored by git by default.

Expected runtime layout:

```text
maps/<map_name>/
  map.yaml
  map.pgm
  metadata.json
  slam_posegraph*   # optional, when slam_toolbox serialization succeeds
```

The project tracks this README and `.gitkeep`, but not generated apartment/room maps.
