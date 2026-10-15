# Helpers for the CI scripts; source it, do not run it.

# stop PID [SIGNAL] [SECONDS]: send SIGNAL (default TERM) to the process, or to
# its process group when it leads one, wait up to SECONDS (default 30) for it
# to exit, then kill it. A node that does not exit on its signal (seen once
# with rclpy on Jazzy, after the work was done) must not hang the job.
stop() {
  local pid=$1 sig=${2:-TERM} secs=${3:-30} target=$1
  kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
  if [ "$(ps -o pgid= -p "$pid" | tr -d ' ')" = "$pid" ]; then
    target=-$pid
  fi
  kill -"$sig" -- "$target" 2>/dev/null || true
  for _ in $(seq $((secs * 5))); do
    kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null || true; return 0; }
    sleep 0.2
  done
  echo "process $pid did not exit within $secs s of SIG$sig; killing it"
  kill -KILL -- "$target" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
