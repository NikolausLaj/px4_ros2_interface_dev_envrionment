#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_DIR=$(cd -- "${SCRIPT_DIR}/.." && pwd)

GUI_MODE=auto
NVIDIA=false
DRY_RUN=false

usage() {
    cat <<EOF
Usage: $0 [--gui | --no-gui] [--nvidia] [--dry-run]

Options:
  --gui      Enable Linux X11 GUI forwarding (the default on Linux).
  --no-gui   Run headless and expose PX4's UDP port for host QGroundControl.
  --nvidia   Use the NVIDIA container runtime (Linux only).
  --dry-run  Print the docker command without executing it.
  -h, --help Show this help.

On macOS, headless mode is selected automatically. Run QGroundControl and
Foxglove natively on the Mac; Gazebo continues to run headless in the container.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)
            GUI_MODE=gui
            ;;
        --no-gui)
            GUI_MODE=headless
            ;;
        --nvidia)
            NVIDIA=true
            ;;
        --dry-run)
            DRY_RUN=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
    shift
done

HOST_OS=$(uname -s)

if [[ "${GUI_MODE}" == auto ]]; then
    if [[ "${HOST_OS}" == Darwin ]]; then
        GUI_MODE=headless
    else
        GUI_MODE=gui
    fi
fi

if [[ "${HOST_OS}" == Darwin && "${GUI_MODE}" == gui ]]; then
    echo "Container GUI mode is not supported on macOS." >&2
    echo "Run without --gui and use native QGroundControl and Foxglove." >&2
    exit 1
fi

if [[ "${NVIDIA}" == true && "${HOST_OS}" != Linux ]]; then
    echo "--nvidia is only supported by Docker on Linux hosts." >&2
    exit 1
fi

DOCKER_CMD=(docker run -it --rm)

if [[ "${GUI_MODE}" == gui ]]; then
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "DISPLAY is not set. Configure X11 or use --no-gui." >&2
        exit 1
    fi

    DOCKER_CMD+=(
        --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly
        --env "DISPLAY=${DISPLAY}"
    )

    if [[ "${NVIDIA}" == false && -e /dev/dri ]]; then
        DOCKER_CMD+=(--device /dev/dri:/dev/dri)
    elif [[ "${NVIDIA}" == false ]]; then
        echo "Warning: /dev/dri is unavailable; using software rendering." >&2
        DOCKER_CMD+=(--env LIBGL_ALWAYS_SOFTWARE=1)
    fi
else
    DOCKER_CMD+=(--publish 18570:18570/udp)
fi

if [[ "${NVIDIA}" == true ]]; then
    DOCKER_CMD+=(
        --runtime nvidia
        --env NVIDIA_VISIBLE_DEVICES=all
        --env NVIDIA_DRIVER_CAPABILITIES=all
    )
fi

DOCKER_CMD+=(
    --publish 8765:8765
    --mount "type=bind,source=${REPOSITORY_DIR},target=/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop"
    --name px4-roscon-25
    --workdir /home/ubuntu/roscon-25-workshop_ws
    dronecode/roscon-25-workshop:latest
    bash
)

if [[ "${DRY_RUN}" == true ]]; then
    printf '%q ' "${DOCKER_CMD[@]}"
    printf '\n'
else
    exec "${DOCKER_CMD[@]}"
fi
