#!/usr/bin/env bash

# Define source and destination directories
SOURCE_DATA_DIR="/workspace/autoware_asset/autoware_data"
DEST_DATA_DIR="/home/cyn/autoware_data"
SOURCE_MAP_DIR="/workspace/autoware_asset/autoware_map"
DEST_MAP_DIR="/home/cyn/autoware_map"

# Function: Check directory and copy
copy_directory() {
  local SOURCE_DIR="$1"
  local DEST_DIR="$2"

  if [ -d "$SOURCE_DIR" ]; then
    echo "Directory $SOURCE_DIR exists, copying to $DEST_DIR..."
    
    # Check if the destination directory exists
    if [ ! -d "$DEST_DIR" ]; then
      cp -r "$SOURCE_DIR" "$DEST_DIR" && echo "Copy successful!" || echo "Copy failed, please check permissions or directory paths."
    else
      echo "Destination directory $DEST_DIR already exists, skipping copy."
    fi
  else
    echo "Directory $SOURCE_DIR does not exist, unable to copy."
  fi
}

# Call the function to handle the data and map directories
copy_directory "$SOURCE_DATA_DIR" "$DEST_DATA_DIR"
copy_directory "$SOURCE_MAP_DIR" "$DEST_MAP_DIR"
