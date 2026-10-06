# One shelf in a bay's vertical stack. Used by DetectBinsOnShelf.srv and
# GetShelfPlane.srv so the camera and the warehouse agree on which z-range
# corresponds to which shelf when a single request covers multiple shelves
# (the warehouse partitions returned BinDetection[] by detection.center.z
# against these brackets).
#
# bottom_z already includes the shelf-front beam, matching the convention
# used by warehouse.yaml's `shelf_bottom_heights_m + shelf_beam_width`.
string shelf_id        # e.g. "A", "B", "C"
float32 bottom_z       # m
float32 top_z          # m
