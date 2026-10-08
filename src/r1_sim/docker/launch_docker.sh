#!/usr/bin/env bash
set -euo pipefail

# Build and start the Gazebo fleet-manager container.
# Mirrors r1_startup/launch_scripts/launch_docker.sh: one image, one named
# container, host networking so the fleet manager shares the ROS graph.
#
# Usage:
#   ./docker/launch_docker.sh [--build-image] [--gui] [--attach]
#
#   --build-image   Rebuild r1-sim:jazzy before starting.
#   --gui           Open Gazebo and RViz on this machine's display.
#   --attach        Follow the container logs after start.
#
# The fleet manager must use the same ROS_DOMAIN_ID. Headless mode does not
# send a test goal; the fleet manager sends NavigateToPose.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${R1_IMAGE_NAME:-r1-sim:jazzy}"
CONTAINER_NAME="${R1_CONTAINER_NAME:-gazebo_sim}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

BUILD_IMAGE=false
GUI=false
ATTACH=false
for arg in "$@"; do
    case "$arg" in
        --build-image) BUILD_IMAGE=true ;;
        --gui)         GUI=true ;;
        --attach)      ATTACH=true ;;
        *) echo "Unknown option: $arg"; echo "Usage: $0 [--build-image] [--gui] [--attach]"; exit 1 ;;
    esac
done

if ! docker images --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}$"; then
    echo "Image '${IMAGE_NAME}' not found locally — it will be built."
    BUILD_IMAGE=true
fi

if [ "$BUILD_IMAGE" = true ]; then
    echo "Building ${IMAGE_NAME}"
    docker build --tag "${IMAGE_NAME}" "${PACKAGE_DIR}"
fi

if [ "$(docker ps -aq -f "name=^/${CONTAINER_NAME}$")" ]; then
    echo "Removing existing container '${CONTAINER_NAME}'..."
    docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

RUN_ARGS=(
    -d
    --name "${CONTAINER_NAME}"
    --net=host
    --ipc=host
    -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    -e "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
)

MODE="headless"
if [ "$GUI" = true ]; then
    MODE="gui"
    xhost +local:docker >/dev/null 2>&1 || true
    RUN_ARGS+=(
        -e "DISPLAY=${DISPLAY:-:0}"
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix
    )
fi

echo "Starting ${CONTAINER_NAME} (${MODE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
docker run "${RUN_ARGS[@]}" "${IMAGE_NAME}" "${MODE}"

echo "Container '${CONTAINER_NAME}' started."
echo "  docker logs -f ${CONTAINER_NAME}"
if [ "$ATTACH" = true ]; then
    docker logs -f "${CONTAINER_NAME}"
fi
