#!/usr/bin/env bash
# EROAS simulation helper: builds the Docker environment and runs the World A
# demo with the Gazebo window and RViz. See SETUP.md.
#
#   ./run_eroas.sh setup              build the image and the workspace (first time)
#   ./run_eroas.sh start [options]    start the simulation (Gazebo window + RViz)
#   ./run_eroas.sh teleop             stop EROAS and drive the vehicle by keyboard
#   ./run_eroas.sh stop               stop the simulation
#   ./run_eroas.sh help               all commands and options
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${EROAS_IMAGE:-eroas:jazzy}"
CONTAINER="${EROAS_CONTAINER:-eroas}"
# GPU architectures for the DAVE sonar CUDA code: Pascal, Turing, Ampere
# (+ PTX for newer GPUs). Override e.g. EROAS_CUDA_ARCH=89
CUDA_ARCH="${EROAS_CUDA_ARCH:-60-real;75-real;86}"
WS=/root/eroas_ws
SRC="$WS/src/EROAS"
LOG=/root/runs/launch.log
ROS_ENV="source /opt/ros/jazzy/setup.bash; [ -f $WS/install/local_setup.bash ] && source $WS/install/local_setup.bash"

# Re-run under the docker group if this login session predates joining it
if ! docker info >/dev/null 2>&1 && id -nG "$USER" | grep -qw docker && [ -z "${EROAS_SG:-}" ]; then
  exec sg docker -c "EROAS_SG=1 $(printf '%q ' "$0" "$@")"
fi

die() { echo "error: $*" >&2; exit 1; }
info() { echo "==> $*"; }

in_ctr() { docker exec "$CONTAINER" bash -c "$ROS_ENV; $1"; }
exists() { docker container inspect "$CONTAINER" >/dev/null 2>&1; }
running() { [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ]; }
# [x] in pgrep/pkill patterns stops them matching the bash -c that runs them
sim_running() { in_ctr "pgrep -f '[r]os2 launch rexrov2_gazebo' >/dev/null"; }

check_host() {
  command -v docker >/dev/null || die "Docker is not installed (see SETUP.md, step 2)"
  docker info >/dev/null 2>&1 || die "cannot talk to Docker; add yourself to the docker group and log in again (SETUP.md, step 2)"
  command -v nvidia-smi >/dev/null || die "no NVIDIA driver found (SETUP.md, step 1)"
}

ensure_container() {
  check_host
  if ! exists; then
    docker image inspect "$IMAGE" >/dev/null 2>&1 || die "image $IMAGE not built yet; run: $0 setup"
    info "creating container $CONTAINER"
    docker run -d --name "$CONTAINER" --gpus all --net host --ipc host \
      -e DISPLAY="${DISPLAY:-:0}" -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$REPO":"$SRC" "$IMAGE" >/dev/null
  elif ! running; then
    info "starting container $CONTAINER"
    docker start "$CONTAINER" >/dev/null
  fi
  in_ctr "nvidia-smi -L >/dev/null" || die "the container cannot see the GPU (SETUP.md, step 3), or try: docker restart $CONTAINER"
}

cmd_image() {
  check_host
  info "building image $IMAGE (downloads several GB the first time)"
  docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO"
}

cmd_build() {
  ensure_container
  in_ctr "test -d $SRC" || die "container $CONTAINER has no source mount at $SRC; copy changed files in with docker cp"
  info "building the workspace in $CONTAINER"
  in_ctr "cd $WS && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CUDA_ARCHITECTURES=$CUDA_ARCH'"
}

cmd_start() {
  local world=world_a headless=0 extra=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --headless) headless=1 ;;
      --world) world="${2:?--world needs a name}"; shift ;;
      *:=*) extra+=("$1") ;;
      *) die "unknown option $1 (see $0 help)" ;;
    esac
    shift
  done
  ensure_container
  sim_running && die "the simulation is already running; use '$0 stop' first"
  in_ctr "test -f $WS/install/local_setup.bash" || die "workspace not built; run: $0 build"

  info "launching EROAS in $world"
  docker exec -d "$CONTAINER" bash -c "$ROS_ENV; mkdir -p /root/runs; \
    ros2 launch rexrov2_gazebo start_EROAS_demo.launch.py gui:=false world_name:=$world ${extra[*]} > $LOG 2>&1"

  info "waiting for RexROV2 to spawn"
  local t=0
  until in_ctr "grep -q 'Entity creation successful' $LOG 2>/dev/null"; do
    sleep 2; t=$((t + 2))
    sim_running || { in_ctr "tail -20 $LOG"; die "the launch exited; log above"; }
    [ $t -lt 180 ] || die "vehicle did not spawn within 180 s; see: $0 logs"
  done
  info "vehicle spawned after ${t} s"
  [ $headless = 1 ] && { info "running headless; '$0 stop' to stop"; return; }

  # The GUI must start after the spawn, or it never shows the vehicle. It
  # needs the server's resource path to find the models.
  command -v xhost >/dev/null && xhost +SI:localuser:root >/dev/null || true
  docker exec -d -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c "$ROS_ENV; \
    export \$(tr '\0' '\n' < /proc/\$(pgrep -of '[g]z sim')/environ | grep ^GZ_SIM_RESOURCE_PATH=); \
    gz sim -g --force-version 8 > /root/runs/gui.log 2>&1"
  docker exec -d -e DISPLAY="${DISPLAY:-:0}" "$CONTAINER" bash -c "$ROS_ENV; \
    r=$SRC/tools/sonar.rviz; [ -f \$r ] || r=/root/sonar.rviz; \
    rviz2 -d \$r > /root/runs/rviz.log 2>&1"
  info "Gazebo window and RViz opening; '$0 teleop' to drive by hand, '$0 stop' to stop"
}

cmd_teleop() {
  running && sim_running || die "the simulation is not running; run: $0 start"
  info "stopping the EROAS navigation nodes so they don't fight the keyboard"
  in_ctr "pkill -f '[r]os1_only_gap|[r]os1_velocity_cbf|[o]nly_gap.py|[v]elocity_cbf.py'" || true
  docker exec -it "$CONTAINER" bash -c "$ROS_ENV; \
    t=$SRC/tools/teleop.py; [ -f \$t ] || t=/root/teleop.py; python3 \$t"
}

cmd_stop() {
  running || { info "container $CONTAINER is not running"; return; }
  info "stopping the simulation"
  in_ctr "pkill -INT -f '[r]os2 launch'; pkill -f '[r]viz2'; pkill -f '[g]z sim'; pkill -f '[t]eleop.py'" || true
  sleep 3
  in_ctr "pkill -9 -f '[g]z sim|[r]viz2|[r]os2 launch|[r]os1_|[p]arameter_bridge|[r]obot_state_publisher'" || true
  info "stopped"
}

cmd_down() { cmd_stop; exists && running && docker stop "$CONTAINER" >/dev/null; info "container $CONTAINER stopped"; }
cmd_shell() { ensure_container; docker exec -it "$CONTAINER" bash; }
cmd_logs() { running || die "container $CONTAINER is not running"; docker exec "$CONTAINER" tail -n 40 -f "$LOG"; }

cmd_help() {
  cat <<EOF
Usage: $0 <command> [options]

  setup                 image + build (first time; takes a while)
  image                 build the Docker image $IMAGE (dependencies only)
  build                 build the ROS 2 workspace in container $CONTAINER (after code changes)
  start [options]       start the simulation, then the Gazebo window and RViz
      --world NAME        world_a (default), world_a_figure5, world_b, blue_blocks
      --headless          no Gazebo window or RViz
      name:=value         any launch argument, e.g. navigation:=port sonar_source:=lidar
  teleop                stop the EROAS nodes and drive by keyboard
  stop                  stop the simulation (container keeps running)
  down                  stop the simulation and the container
  logs                  follow the launch log
  shell                 bash inside the container

Environment: EROAS_CONTAINER (default eroas), EROAS_IMAGE (default eroas:jazzy),
EROAS_CUDA_ARCH (default "$CUDA_ARCH").
EOF
}

case "${1:-help}" in
  setup) cmd_image; cmd_build ;;
  image|build|start|teleop|stop|down|shell|logs|help) c="$1"; shift; "cmd_$c" "$@" ;;
  -h|--help) cmd_help ;;
  *) die "unknown command $1 (see $0 help)" ;;
esac
