cmake_minimum_required(VERSION 3.8)
project(r1_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunInference.action"
  "action/ServoToTarget.action"
  "action/SendPickTicket.action"
  "action/ScanForBarcode.action"
  "action/SolveIK.action"
  "action/HighlightTarget.action"
  "action/ServoHeight.action"
  "action/StrafeToPose.action"
  "action/ScanBarcodes.action"
  "srv/GetChassisTarget.srv"
  "srv/GetStrafeScanPlan.srv"
  "srv/GetRawBinPosition.srv"
  "srv/CompleteJob.srv"
  "srv/GetCurrentPickTicket.srv"
  "srv/GetPickQueue.srv"
  "srv/RemovePick.srv"
  "srv/AbortAndHome.srv"
  "srv/LookupByBarcode.srv"
  "srv/UpdateLocationDetection.srv"
  "srv/CreateCart.srv"
  "srv/GetCartInfo.srv"
  "srv/FinalizeCart.srv"
  "srv/ScanTote.srv"
  "srv/ListCarts.srv"
  "srv/ConfirmPick.srv"
  "srv/RequestNextPick.srv"
  "srv/ClearCart.srv"
  "srv/DeleteCart.srv"
  "srv/SetupCart.srv"
  "srv/GetDefaultOrderStatusFilter.srv"
  "srv/ReleaseLocks.srv"
  "srv/SkipPick.srv"
  "srv/TryNextLocation.srv"
  "srv/ListAlternateLocations.srv"
  "srv/SelectPickLocation.srv"
  "srv/DetectObjects.srv"
  "srv/DetectBinsOnShelf.srv"
  "srv/RecordBinDetections.srv"
  "srv/GetTargetBoundingBoxes.srv"
  "srv/GetShelfPlane.srv"
  "msg/Bin.msg"
  "msg/BinDetection.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/ShelfBracket.msg"
  "msg/Tote.msg"
  "msg/PickTicket.msg"
  "msg/TagDetections.msg"
  "msg/BarcodeDetection.msg"
  "msg/BarcodeReading.msg"
  "msg/BarcodeScan.msg"
  "msg/WarehouseLocation.msg"
  "msg/HealthCode.msg"
  "msg/HealthStatus.msg"
  "msg/OperatorAlert.msg"
  "msg/BTState.msg"
  "msg/CollisionState.msg"
  "msg/AlternateLocation.msg"
  "msg/FleetRobotState.msg"
  "msg/FleetNavRequest.msg"
  "msg/FleetActionRequest.msg"
  "msg/FleetModeRequest.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_package()
