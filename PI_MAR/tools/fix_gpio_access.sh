#!/bin/bash
# Run on Raspberry Pi HOST as user pi, not inside Docker. No network required.
set -euo pipefail
container=${1:-MentorPi}
account=${2:-ubuntu}
if [ -e /.dockerenv ]; then
  echo 'Run this script on Pi HOST after SSH, not inside Docker.' >&2
  exit 1
fi
command -v docker >/dev/null
test "$(docker inspect -f '{{.State.Running}}' "$container")" = true
gids=()
declare -A seen=()
for device in /dev/gpiochip*; do
  [ -c "$device" ] || continue
  resolved=$(readlink -f -- "$device")
  [ -z "${seen[$resolved]+x}" ] || continue
  seen[$resolved]=1
  gid=$(stat -c '%g' -- "$resolved")
  mode=$(stat -c '%a' -- "$resolved")
  if [ "$gid" = 0 ]; then
    echo "Refusing to add $account to root group for $resolved. Check host udev GPIO rule." >&2
    exit 1
  fi
  docker exec "$container" test -c "$resolved"
  docker_gid=$(docker exec "$container" stat -c '%g' -- "$resolved")
  if [ "$docker_gid" != "$gid" ]; then
    echo "GPIO group differs between host and Docker at $resolved; inspect device mapping." >&2
    exit 1
  fi
  printf 'GPIO %s gid=%s mode=%s\n' "$resolved" "$gid" "$mode"
  gids+=("$gid")
done
if [ "${#gids[@]}" = 0 ]; then
  echo 'No GPIO character devices found on host.' >&2
  exit 1
fi
for gid in $(printf '%s\n' "${gids[@]}" | sort -un); do
  docker exec -i -u root "$container" /bin/sh -s -- "$gid" "$account" <<'CONTAINER'
set -eu
gpio_gid=$1
gpio_account=$2
id "$gpio_account" >/dev/null
gpio_group=$(getent group "$gpio_gid" | cut -d: -f1)
if [ -z "$gpio_group" ]; then
  gpio_group="mediassist_gpio_${gpio_gid}"
  if getent group "$gpio_group" >/dev/null; then
    echo "Group name exists with another GID: $gpio_group" >&2
    exit 1
  fi
  groupadd --gid "$gpio_gid" "$gpio_group"
fi
usermod -aG "$gpio_group" "$gpio_account"
printf 'Added %s to %s (GID %s).\n' "$gpio_account" "$gpio_group" "$gpio_gid"
CONTAINER
done
docker exec -u "$account" "$container" id
echo 'Open a NEW docker exec -it -u ubuntu session. Existing terminals keep old groups.'
echo 'GPIO group membership will need restoring if MentorPi container is recreated.'
