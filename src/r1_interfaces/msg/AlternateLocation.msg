# Description of one pickable location returned by ListAlternateLocations.
# All fields are populated by the WMS; the GUI renders them verbatim.

string location_name      # Bin name as displayed (e.g., "Y50-03-B-02")
string location_id        # Stable bin identifier (passed back in SelectPickLocation)
int32  quantity           # Total on-hand qty at this bin (>= 1 for returned rows)
int32  remaining          # Remaining qty after current batch allocations
int32  pick_priority      # Provider-assigned pick priority (higher = preferred)
string row                # Row component (e.g., "Y50")
string bay                # Bay component (e.g., "03")
string shelf              # Shelf component (e.g., "B")
string bin                # Bin component (e.g., "02")
string assigned_robot     # Robot the WMS would dispatch this bin to (may be "" if unassigned)
bool   in_robot_zone      # True if the calling robot has an EXPLICIT (non-wildcard)
                          # robot_assignment rule match for this location
bool   is_current         # True if this row is the location currently dispatched
