# A single health code entry for one robot subsystem category.
# Populated from health_codes.yaml on the robot. Used as a field inside
# HealthStatus to describe control, battery, and navigation independently.

uint16 id                  # Numeric health code (e.g. 100, 201, 301)
string description         # Short description (e.g. "Healthy", "Battery Low")
string robot_status        # Operational impact (e.g. "Fully operational")
string diagnosis_process   # Explanation of how the code was determined
