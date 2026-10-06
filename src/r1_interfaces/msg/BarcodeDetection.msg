# Custom Message for Publishing barcodes detected by the scanner
std_msgs/Header header
string barcode_id

# Scanner pose at time of detection (in map frame)
# Used by tag_detections_node to compute barcode position via ray-line intersection
geometry_msgs/PoseStamped scanner_pose
