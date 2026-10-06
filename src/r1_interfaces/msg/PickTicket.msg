# Custom message containing all info for a single WMS pick
# Sent from shiphero_wms to teleop station

# Order identification
string robot_name
string order_id
string order_number

# Item details
string item_name
string sku
int64 quantity
string line_item_id              # Unique line item ID (shipment_line_item_id)
string pick_token               # Unique dispatch instance token

# Pick location (bin)
string pick_arm
r1_interfaces/Bin bin
int64 bin_tag_id
string bin_tag_txt

# Tote destination
r1_interfaces/Tote tote          # row/col position on cart
int64 tote_tag_id
string tote_tag_txt
string tote_name                  # ShipHero tote name (e.g., "Tote-221")
string tote_barcode               # Physical tote barcode

# Cart info
string cart_barcode               # Cart this tote is on

# Batch progress info
string batch_id                   # ShipHero batch ID
int32 pick_number                 # Current pick number (1-indexed)
int32 total_picks_in_order        # Total picks for this order
int32 remaining_picks_in_batch    # Remaining picks in entire batch
int32 total_orders_in_batch       # Total orders in batch
int32 remaining_orders_in_batch   # Orders still to complete

# Atomic SIB safety context (additive; zero/empty defaults preserve legacy MIB)
string batch_type
string sib_product_id
string sib_group_location_id
uint32 sib_group_quantity
uint32 sib_group_member_index
uint32 sib_group_confirmed_count
bool location_change_allowed
string location_change_block_reason
uint64 state_revision             # Monotonic within one pick_token
