#!/usr/bin/env bash

# Enable error handling (script will stop if any command fails)
set -e

# Customize the shell prompt with colors
export PS1="\[\033[1;32m\]\u\[\033[1;35m\][autoware]\[\033[0m\] \[\033[1;34m\]\w\[\033[0m\] > "

# Function to execute a script
# Parameter 1: Script path
# Parameter 2: Execution type (bash or source)
run_script() {
  local SCRIPT_PATH="$1"
  local EXEC_TYPE="$2"

  if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Script $SCRIPT_PATH does not exist!" >&2
    exit 1
  fi

  echo "Executing $SCRIPT_PATH ..."

  case "$EXEC_TYPE" in
    bash)
      bash "$SCRIPT_PATH"
      ;;
    *)
      echo "Unsupported execution type: $EXEC_TYPE" >&2
      exit 1
      ;;
  esac

  if [ $? -eq 0 ]; then
    echo "$SCRIPT_PATH executed successfully!"
  else
    echo "$SCRIPT_PATH execution failed!" >&2
    exit 1
  fi
}

# Define an associative array of scripts and their execution types
declare -A SCRIPTS_TO_RUN

# Add script paths and their execution types
SCRIPTS_TO_RUN["/workspace/my_scripts/cp_data.sh"]="bash"

# Iterate over the scripts and execute them
for SCRIPT_PATH in "${!SCRIPTS_TO_RUN[@]}"; do
  EXEC_TYPE="${SCRIPTS_TO_RUN[$SCRIPT_PATH]}"
  echo
  run_script "$SCRIPT_PATH" "$EXEC_TYPE"
done

echo "custom_set_up.sh execution completed!"
