# Message describing a warehouse location with detection state
string location_key           # e.g., "X2-01-A-1" for bins, "X2-01" for totes
string location_type          # "BIN" or "TOTE"
string barcode                # Expected barcode at this location

# Poses
geometry_msgs/Pose base_pose      # Static pose from YAML config
geometry_msgs/Pose refined_pose   # Refined pose from detections (if available)

# Detection state
bool detected                 # Has this location been confirmed by detection
float32 confidence            # 0.0-1.0 joint position+width confidence
int32 detection_count         # Number of times detected
builtin_interfaces/Time last_detection  # When last detected

# Width belief (BIN locations only; 0 for TOTE)
float32 width_estimate        # Posterior width mean (m)
float32 width_stddev          # Posterior width std-dev (m)
