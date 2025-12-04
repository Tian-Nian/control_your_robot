#!/bin/bash

# Data processing script - Convert HDF5 data to zarr format for Diffusion Policy training

# Get arguments
SOURCE_DIR="$1"
OUTPUT_DIR="$2"
NUM_EPISODES="$3"

# Execute Python script
python3 scripts/process_data.py "$SOURCE_DIR" "$OUTPUT_DIR" "$NUM_EPISODES"
