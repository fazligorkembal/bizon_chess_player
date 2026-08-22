#!/usr/bin/env bash
# PreToolUse/Bash hook: refuse raw colcon build and colcon test invocations and
# point at scripts/rosbuild instead.
#
# A raw colcon run prints thousands of lines straight into the agent's context,
# which is the largest avoidable waste in this repository. rosbuild runs the same
# command and prints only the verdict or the decisive errors, keeping the full log
# on disk. The wrapper calls colcon inside a container, so its own invocation never
# reaches this hook; commands that mention rosbuild are passed through regardless.
#
# Reads the hook payload on stdin and prints a deny decision when it should block.
# Printing nothing lets the command proceed.

set -uo pipefail

command_text="$(jq -r '.tool_input.command // ""')"

case "$command_text" in
  *rosbuild*) exit 0 ;;
esac

if ! printf '%s' "$command_text" | grep -qE '(^|[^[:alnum:]_./-])colcon[[:space:]]+(build|test)([[:space:]]|$)'; then
  exit 0
fi

reason='Raw colcon output floods the context. Use scripts/rosbuild instead: it runs the same build or test inside the project image and prints only the verdict, or the failing packages and the decisive errors on failure, with the full log kept at ~/.cache/bizon-build/last.log. Arguments pass straight through, so "colcon build --packages-select X" becomes "scripts/rosbuild --packages-select X", and "colcon test" becomes "scripts/rosbuild --test". Run "scripts/rosbuild --explain" to have the local model root-cause the last failure, or read the log file directly if the summary is genuinely not enough.'

jq -nc --arg reason "$reason" '{
  hookSpecificOutput: {
    hookEventName: "PreToolUse",
    permissionDecision: "deny",
    permissionDecisionReason: $reason
  }
}'
