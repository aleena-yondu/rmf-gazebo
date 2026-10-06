#!/usr/bin/env bash
# =============================================================================
# record_rotation_calib.sh
#
# Records IMU + LiDAR bags at multiple rotation speeds for gyro z-axis bias
# calibration. Works with calibrate_rotation_bias_multi.py afterwards.
#
# Usage:
#   bash record_rotation_calib.sh [OUTPUT_DIR]          # interactive picker
#   bash record_rotation_calib.sh [OUTPUT_DIR] --all    # record all 10
#   bash record_rotation_calib.sh [OUTPUT_DIR] --missing # only missing bags
#
# Requirements:
#   - ROS2 workspace sourced
#   - synchro_drive_odometry + cmd_vel_relay running
#   - Ouster driver running
#   - Robot in open space with >= 1m clearance all around
# =============================================================================

set -euo pipefail

# ── Config ────────────────────────────────────────────────────────────────────

SPEEDS=(0.3 0.5 0.7 1.0 1.2)
DURATION=20       # hold time at steady speed (s)
RAMP_TIME=2.0     # ramp-up / ramp-down (s)
POST_NUDGE=0.2    # forward/backward nudge after each rotation (m); 0 = disabled
RECORD_TOPICS="/ouster/imu /ouster/points /synchro_odom"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STEADY_ROTATE="${SCRIPT_DIR}/steady_rotate.py"

BAG_DIR="${1:-$HOME/calib_bags/$(date +%Y%m%d_%H%M%S)}"
MODE="${2:-pick}"   # pick | all | missing

# ── Helpers ───────────────────────────────────────────────────────────────────

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; CYN='\033[0;36m'
DIM='\033[2m'; NC='\033[0m'

ROT_PID=""; REC_PID=""

cleanup() {
    echo ""
    echo -e "${YLW}[interrupt] stopping...${NC}"
    [ -n "$ROT_PID" ] && kill "$ROT_PID" 2>/dev/null || true
    _stop_recording
    echo -e "${RED}Aborted. Partial bags may exist in: $BAG_DIR${NC}"
    exit 1
}
trap cleanup INT TERM

_stop_recording() {
    if [ -n "$REC_PID" ]; then
        kill -INT "$REC_PID" 2>/dev/null || true
        wait "$REC_PID" 2>/dev/null || true
        REC_PID=""
    fi
}

_wait_user() {
    echo -e "${YLW}$1${NC}"
    read -rp "  → Press ENTER to continue... "
}

_bag_exists() {   # $1 = bag path
    [ -d "$1" ] && ls "$1"/*.db3 >/dev/null 2>&1
}

# ── Build full candidate list ─────────────────────────────────────────────────
# ALL_BAGS[i] = "SPEED DIR"
ALL_BAGS=()
for S in "${SPEEDS[@]}"; do
    ALL_BAGS+=("$S cw")
    ALL_BAGS+=("$S ccw")
done

# ── Header ────────────────────────────────────────────────────────────────────

mkdir -p "$BAG_DIR"

echo ""
echo -e "${CYN}════════════════════════════════════════════════════════${NC}"
echo -e "${CYN}  Rotation Bias Calibration — Multi-Speed Bag Recorder  ${NC}"
echo -e "${CYN}════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  Output dir : ${GRN}$BAG_DIR${NC}"
echo -e "  Duration   : ${DURATION}s hold + ${RAMP_TIME}s ramp each side"
echo -e "  SLAM nudge : ±${POST_NUDGE}m after each rotation"
echo ""

# Show status of all bags
echo "  All bags:"
IDX=0
for ENTRY in "${ALL_BAGS[@]}"; do
    IDX=$(( IDX + 1 ))
    read -r S DIR <<< "$ENTRY"
    BAG_NAME="rot_${S//./_}_${DIR}"
    BAG_PATH="${BAG_DIR}/${BAG_NAME}"
    ARROW="↻ CW "; [ "$DIR" = "ccw" ] && ARROW="↺ CCW"
    if _bag_exists "$BAG_PATH"; then
        echo -e "  ${DIM}[$IDX]${NC}  ${GRN}✓${NC}  ${ARROW} @ ${S} rad/s  ${DIM}(exists)${NC}"
    else
        echo -e "  ${DIM}[$IDX]${NC}  ${RED}✗${NC}  ${ARROW} @ ${S} rad/s"
    fi
done
echo ""

# ── Select which bags to record ───────────────────────────────────────────────

SELECTED=()   # indices (1-based) into ALL_BAGS

case "$MODE" in
    --all)
        for i in $(seq 1 ${#ALL_BAGS[@]}); do SELECTED+=("$i"); done
        echo -e "  Mode: ${YLW}record all${NC}"
        ;;
    --missing)
        IDX=0
        for ENTRY in "${ALL_BAGS[@]}"; do
            IDX=$(( IDX + 1 ))
            read -r S DIR <<< "$ENTRY"
            BAG_PATH="${BAG_DIR}/rot_${S//./_}_${DIR}"
            if ! _bag_exists "$BAG_PATH"; then
                SELECTED+=("$IDX")
            fi
        done
        echo -e "  Mode: ${YLW}missing bags only${NC} (${#SELECTED[@]} bags)"
        ;;
    *)
        # Interactive picker
        echo -e "  ${YLW}Which bags to record?${NC}"
        echo "  Enter numbers separated by spaces, or shortcuts:"
        echo -e "  ${CYN}a${NC} = all   ${CYN}m${NC} = missing only   ${CYN}e.g.: 3 4 7 8${NC}"
        echo ""
        read -rp "  > " CHOICE

        if [ "$CHOICE" = "a" ]; then
            for i in $(seq 1 ${#ALL_BAGS[@]}); do SELECTED+=("$i"); done
        elif [ "$CHOICE" = "m" ]; then
            IDX=0
            for ENTRY in "${ALL_BAGS[@]}"; do
                IDX=$(( IDX + 1 ))
                read -r S DIR <<< "$ENTRY"
                BAG_PATH="${BAG_DIR}/rot_${S//./_}_${DIR}"
                if ! _bag_exists "$BAG_PATH"; then
                    SELECTED+=("$IDX")
                fi
            done
        else
            for N in $CHOICE; do
                if [[ "$N" =~ ^[0-9]+$ ]] && [ "$N" -ge 1 ] && [ "$N" -le "${#ALL_BAGS[@]}" ]; then
                    SELECTED+=("$N")
                else
                    echo -e "  ${RED}Ignored invalid entry: $N${NC}"
                fi
            done
        fi
        ;;
esac

if [ ${#SELECTED[@]} -eq 0 ]; then
    echo -e "\n  ${GRN}Nothing to record. Exiting.${NC}"
    exit 0
fi

# Print plan
echo ""
echo -e "  Will record ${YLW}${#SELECTED[@]} bag(s)${NC}:"
for N in "${SELECTED[@]}"; do
    read -r S DIR <<< "${ALL_BAGS[$(( N - 1 ))]}"
    ARROW="↻ CW "; [ "$DIR" = "ccw" ] && ARROW="↺ CCW"
    echo -e "    • ${ARROW} @ ${S} rad/s  →  rot_${S//./_}_${DIR}"
done
echo ""

_wait_user "Ready? Make sure the robot has clear space around it."

# ── Recording loop ────────────────────────────────────────────────────────────

DONE=0
TOTAL=${#SELECTED[@]}

for N in "${SELECTED[@]}"; do
    DONE=$(( DONE + 1 ))
    read -r SPEED DIR <<< "${ALL_BAGS[$(( N - 1 ))]}"

    if [ "$DIR" = "cw" ]; then
        OMEGA="-${SPEED}"; LABEL="CW"; ARROW="↻"
    else
        OMEGA="${SPEED}";  LABEL="CCW"; ARROW="↺"
    fi

    BAG_NAME="rot_${SPEED//./_}_${DIR}"
    BAG_PATH="${BAG_DIR}/${BAG_NAME}"

    # Warn if overwriting
    if _bag_exists "$BAG_PATH"; then
        echo ""
        echo -e "  ${YLW}⚠ Bag already exists: ${BAG_NAME}${NC}"
        read -rp "  Overwrite? [y/N] " OW
        if [[ ! "$OW" =~ ^[Yy]$ ]]; then
            echo "  Skipped."
            continue
        fi
        rm -rf "$BAG_PATH"
    fi

    echo ""
    echo -e "${CYN}────────────────────────────────────────────────────────${NC}"
    echo -e "  Bag ${DONE}/${TOTAL}: ${YLW}${ARROW} ${LABEL} @ ${SPEED} rad/s${NC}"
    echo -e "  Saving to: ${BAG_PATH}"
    echo -e "${CYN}────────────────────────────────────────────────────────${NC}"

    _wait_user "Robot STATIONARY? Ready to start recording?"

    # Start recording first
    ros2 bag record -o "$BAG_PATH" $RECORD_TOPICS >/dev/null 2>&1 &
    REC_PID=$!
    sleep 1

    # Run rotation (blocks until done)
    python3 "$STEADY_ROTATE" \
        --speed "$OMEGA" \
        --duration "$DURATION" \
        --ramp-time "$RAMP_TIME" \
        --post-nudge "$POST_NUDGE" &
    ROT_PID=$!
    wait "$ROT_PID" || true
    ROT_PID=""

    _stop_recording

    echo ""
    echo -e "  ${GRN}✓ Bag saved: ${BAG_NAME}${NC}"
    echo ""
    _wait_user "!! Wait for robot to fully stop !! Then confirm stationary:"
done

# ── Summary ───────────────────────────────────────────────────────────────────

echo ""
echo -e "${CYN}════════════════════════════════════════════════════════${NC}"
echo -e "${CYN}  Done! (${DONE}/${TOTAL} bags recorded)${NC}"
echo -e "${CYN}════════════════════════════════════════════════════════${NC}"
echo ""
echo "  Final bag status:"
IDX=0
READY=0
for ENTRY in "${ALL_BAGS[@]}"; do
    IDX=$(( IDX + 1 ))
    read -r S DIR <<< "$ENTRY"
    BAG_NAME="rot_${S//./_}_${DIR}"
    BAG_PATH="${BAG_DIR}/${BAG_NAME}"
    ARROW="↻ CW "; [ "$DIR" = "ccw" ] && ARROW="↺ CCW"
    if _bag_exists "$BAG_PATH"; then
        echo -e "    ${GRN}✓${NC}  ${ARROW} @ ${S} rad/s"
        READY=$(( READY + 1 ))
    else
        echo -e "    ${RED}✗${NC}  ${ARROW} @ ${S} rad/s  ${DIM}(missing)${NC}"
    fi
done
echo ""
echo -e "  ${READY}/${#ALL_BAGS[@]} bags available for calibration."
echo ""
CALIB_CMD="python3 scripts/calibrate_rotation_bias_multi.py \\
    --bag-dir $BAG_DIR \\
    --stationary-bias-z -0.009294 \\
    --save ${BAG_DIR}/result.json"
echo "  Run calibration:"
echo -e "${CYN}"
echo "  $CALIB_CMD"
echo -e "${NC}"



                                                                                
# python3 src/r1_navigation/r1_nav_cpp/scripts/calibrate_rotation_bias_multi.py --bag-dir ./calib/calib_bags/20260317  --stationary-bias-z -0.009294 --save ./calib/calib_bags/20260317/result.json                            
                                                      