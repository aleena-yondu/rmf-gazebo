# One capture frame from the machine-vision barcode scanner: the set of barcodes
# decoded in a single image, sharing one capture timestamp.
#
# header.stamp    = image-capture time reported by the scanner (used by
#                   tag_detections_node to time-sync the TF lookup)
# header.frame_id = the scanner optical frame the pixel coordinates are expressed
#                   in (e.g. "barcode_scanner_optical_frame")
std_msgs/Header header
BarcodeReading[] readings
