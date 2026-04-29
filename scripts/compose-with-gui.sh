#!/usr/bin/env bash
set -euo pipefail

# Prefer the current shell's display when present. Fall back to the local X server
# for host-monitor use on systems where X is listening on :0.
if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  else
    echo "No DISPLAY is set and no local X socket was found at /tmp/.X11-unix/X0." >&2
    exit 1
  fi
fi

# Pass a host-side Xauthority file into the container so Qt apps can authenticate
# against either SSH-forwarded X11 displays or the local desktop session.
if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
  export HOST_XAUTHORITY="${XAUTHORITY}"
elif [[ -f "${HOME}/.Xauthority" ]]; then
  export HOST_XAUTHORITY="${HOME}/.Xauthority"
else
  echo "Could not find an Xauthority file. Set XAUTHORITY before running this script." >&2
  exit 1
fi

# For physical desktop sessions, allow the root user in the container to open windows.
# This is best-effort so SSH-forwarded sessions do not fail here.
if [[ "${DISPLAY}" != localhost:* ]] && command -v xhost >/dev/null 2>&1; then
  xhost +si:localuser:root >/dev/null 2>&1 || true
fi

echo "Using DISPLAY=${DISPLAY}"
echo "Using HOST_XAUTHORITY=${HOST_XAUTHORITY}"

exec docker compose "$@"
