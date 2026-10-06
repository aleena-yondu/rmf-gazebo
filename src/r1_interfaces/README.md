# r1_interfaces

Custom ROS 2 message, service, and action definitions for the R1 robot warehouse picking system.

## Overview

This package defines the communication interfaces between R1 robot subsystems:

- **Messages** - Data structures for orders, warehouse locations, and detections
- **Services** - Request/response interfaces for WMS operations and navigation
- **Actions** - Long-running operations for picking, scanning, and servo motions

---

## Messages

### PickTicket.msg

Complete pick order from the Warehouse Management System (WMS). Contains all information needed to execute a single pick operation.

```
# Order identification
string robot_name              # Target robot
string order_id                # WMS order ID
string order_number            # Human-readable order number

# Item details
string item_name               # Product name
string sku                     # Stock Keeping Unit
int64 quantity                 # Quantity to pick

# Pick location (bin)
string pick_arm                # "left" or "right" arm to use
r1_interfaces/Bin bin          # Bin location details
int64 bin_tag_id               # AprilTag ID on bin
string bin_tag_txt             # Expected bin barcode

# Tote destination
r1_interfaces/Tote tote        # Tote position on cart
int64 tote_tag_id              # AprilTag ID on tote
string tote_tag_txt            # Expected tote barcode
string tote_name               # ShipHero tote name (e.g., "Tote-221")
string tote_barcode            # Physical tote barcode

# Cart info
string cart_barcode            # Cart this tote is on

# Batch progress
string batch_id                # ShipHero batch ID
int32 pick_number              # Current pick (1-indexed)
int32 total_picks_in_order     # Total picks for this order
int32 remaining_picks_in_batch # Remaining in batch
int32 total_orders_in_batch    # Total orders in batch
int32 remaining_orders_in_batch
```

**Used by:** `domain_bridge`, `tag_detections_node`, BT nodes (`UpdateOrderState`, `ServoToTargetAction`, `ScanForBarcodeAction`)

---

### Bin.msg

Warehouse bin location using hierarchical addressing.

```
string side        # "X" or "Y" (aisle side)
int64 row          # Row number
int64 bay          # Bay number within row
string shelf       # Shelf letter ("A" at bottom, ascending)
int64 bin_number   # Bin position on shelf
```

**Example:** Side X, Row 2, Bay 1, Shelf C, Bin 2 → Barcode "X2-01-C-02"

---

### Tote.msg

Tote position on the robot's cart.

```
uint8 row          # Tote row (0-indexed from bottom)
uint8 col          # Tote column (0-indexed from left)
```

---

### BarcodeDetection.msg

Barcode detection event from the Zebra scanner.

```
std_msgs/Header header
string barcode_id                      # Decoded barcode string
geometry_msgs/PoseStamped scanner_pose # Scanner pose at detection time
```

**Used by:** `zebra_barcode_node` publishes to `/barcode`, `tag_detections_node` subscribes to compute bin/tote positions.

---

### TagDetections.msg

Array of AprilTag or fiducial detections.

```
std_msgs/Header header
string detection_type                  # "apriltag", "aruco", etc.
geometry_msgs/PoseStamped[] poses      # Detected tag poses
```

---

### WarehouseLocation.msg

Warehouse location with detection state for navigation refinement.

```
string location_key           # e.g., "X2-01-A-1" for bins
string location_type          # "BIN" or "TOTE"
string barcode                # Expected barcode

# Poses
geometry_msgs/Pose base_pose      # Static pose from YAML config
geometry_msgs/Pose refined_pose   # Refined from detections

# Detection state
bool detected                 # Confirmed by detection
float32 confidence            # 0.0-1.0 detection confidence
int32 detection_count         # Number of detections
builtin_interfaces/Time last_detection
```

---

## Services

### WMS Integration Services

#### RequestNextPick.srv

Request the next pick order for a cart.

```
# Request
string cart_barcode    # Cart to get pick for
string robot_id        # Requesting robot

# Response
bool success
string message
bool pick_available    # True if pick is ready
bool batch_fetched     # True if new batch was fetched
int32 queue_size       # Remaining picks in queue
```

#### ConfirmPick.srv

Confirm a completed pick to WMS.

```
# Request
string cart_barcode
string order_id
string order_number
string sku
int32 qty_picked
string tote_barcode

# Response
bool success
string message
bool order_complete    # Last pick for order
bool batch_complete    # Last pick in batch
int32 remaining_picks
```

#### GetCurrentPickTicket.srv

Query current pick status.

```
# Request (empty)

# Response
bool has_current_order
string order_id
string station_id
bool navigation_complete
bool job_complete
bool is_homing
```

#### GetPickQueue.srv

Get list of pending picks.

```
# Request (empty)

# Response
string[] order_ids
string[] station_ids
int32 queue_length
```

---

### Cart Management Services

#### CreateCart.srv

Create a new cart in the system.

```
# Request
string cart_barcode
string cart_name       # Optional display name
int32 rows             # Default 4
int32 cols             # Default 3

# Response
bool success
string message
string cart_barcode
int32 total_slots
```

#### SetupCart.srv

Setup complete cart with all totes in one call.

```
# Request
string cart_barcode
int32 rows
int32 cols
string[] tote_barcodes  # Row-major order

# Response
bool success
string message
int32 totes_registered
string[] tote_names
```

#### ScanTote.srv

Add a tote to a cart slot.

```
# Request
string cart_barcode
string tote_barcode
int32 row              # -1 for auto
int32 col              # -1 for auto

# Response
bool success
string message
int32 tote_id          # ShipHero tote ID
string tote_name
int32 row              # Actual assigned row
int32 col              # Actual assigned col
```

#### FinalizeCart.srv

Mark cart setup as complete.

```
# Request
string cart_barcode

# Response
bool success
string message
int32 total_totes
string status          # "ready"
```

#### GetCartInfo.srv

Query cart status.

```
# Request
string cart_barcode    # Empty for all carts

# Response
bool success
string status          # setup/ready/in_progress/exhausted
int32 total_totes
int32 available_totes
int32 totes_with_orders
string current_batch_id
bool is_exhausted
```

#### ListCarts.srv

List all carts in system.

```
# Response
string[] cart_barcodes
string[] cart_statuses
int32[] total_totes
int32[] available_totes
```

#### ClearCart.srv

Clear orders from cart totes.

```
# Request
string cart_barcode
bool update_shiphero
string fulfillment_status

# Response
bool success
int32 totes_cleared
string[] cleared_order_ids
```

---

### Navigation Services

#### GetChassisTarget.srv

Get navigation pose for a target type.

```
# Request
string target_type     # "BIN", "TOTE", or "CENTER_AISLE"

# Response
bool detected          # Barcode detected for this location
geometry_msgs/PoseStamped detected_pose   # Barcode-refined pose
geometry_msgs/PoseStamped best_estimate   # YAML-based pose
geometry_msgs/PoseStamped scan_pose       # Pose for barcode scanning
string target_barcode  # Expected barcode (BIN only)
bool success
string message
```

**Used by:** `GetChassisTargetService` BT node

#### LookupByBarcode.srv

Find warehouse location by barcode.

```
# Request
string barcode
float32 timeout        # 0 = use cached

# Response
bool found
r1_interfaces/WarehouseLocation location
string message
```

#### UpdateLocationDetection.srv

Update location state from barcode scan.

```
# Request
string barcode
geometry_msgs/PoseStamped detected_pose

# Response
bool success
string location_key    # e.g., "X2-01-A-1"
string message
```

---

### Robot Control Services

#### AbortAndHome.srv

Abort current operation and return to home.

```
# Request (empty)

# Response
bool success
string message
int32 cleared_tasks
```

#### CompleteJob.srv

Mark a job as complete.

```
# Request
string order_id

# Response
bool success
string message
```

#### RemovePick.srv

Remove a pick from the queue.

```
# Request
string order_id

# Response
bool success
string message
```

---

## Actions

### ScanForBarcode.action

Scan for a specific barcode with timeout.

```
# Goal
string target_barcode
float32 timeout

# Result
bool success           # True if barcode found

# Feedback
float32 time_left      # Remaining scan time
```

**Server:** `zebra_barcode_node`
**Client:** `ScanForBarcodeAction` BT node

---

### ServoToTarget.action

Move arms/torso to access a bin or tote.

```
# Goal
r1_interfaces/PickTicket pick_ticket
string target_type     # "BIN" or "TOTE"

# Result
bool success

# Feedback
string current_sub_state  # Current motion phase
```

---

### RunInference.action

Execute learned pick/place policy.

```
# Goal
int64 steps            # Inference timesteps
int64 sleep            # Ms between steps
string action_type     # "pick" or "place"

# Result
bool success

# Feedback
string message         # Status updates
```

**Server:** Inference node (Python)
**Client:** `RunInferenceBtNode` BT node

---

### SendPickTicket.action

Send robot to a station for a pick operation.

```
# Goal
string station_id
string operation

# Result
bool success
string message

# Feedback
string current_status
float32 distance_remaining
```

---

## Building

```bash
cd ~/yondu_ws
colcon build --packages-select r1_interfaces
source install/setup.bash
```

---

## Usage Examples

### Publishing a PickTicket

```python
from r1_interfaces.msg import PickTicket, Bin, Tote

msg = PickTicket()
msg.robot_name = "terry"
msg.order_id = "ORD-12345"
msg.item_name = "Widget A"
msg.sku = "SKU-001"
msg.quantity = 1
msg.pick_arm = "left"

msg.bin = Bin()
msg.bin.side = "X"
msg.bin.row = 2
msg.bin.bay = 1
msg.bin.shelf = "C"
msg.bin.bin_number = 2

msg.tote = Tote()
msg.tote.row = 1
msg.tote.col = 0

publisher.publish(msg)
```

### Calling GetChassisTarget Service

```python
from r1_interfaces.srv import GetChassisTarget

client = node.create_client(GetChassisTarget, 'get_chassis_target')
request = GetChassisTarget.Request()
request.target_type = "BIN"

future = client.call_async(request)
response = future.result()

if response.success:
    goal_pose = response.best_estimate
    is_detected = response.detected
```

---

## License

Apache-2.0
