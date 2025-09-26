#!/bin/bash

# GPU detection and Docker argument setup script for devcontainer

# Function to detect GPU type
detect_gpu() {
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
        echo "nvidia"
    elif lspci | grep -i amd | grep -i vga >/dev/null 2>&1; then
        echo "amd"
    elif lspci | grep -i intel | grep -i graphics >/dev/null 2>&1; then
        echo "intel"
    else
        echo "generic"
    fi
}

# Get GPU type
GPU_TYPE=$(detect_gpu)

echo "Detected GPU type: $GPU_TYPE"

# Set environment variables based on GPU type
case $GPU_TYPE in
    "nvidia")
        echo "Setting up for NVIDIA GPU"
        export DOCKER_GPU_ARGS="--gpus=all --runtime=nvidia"
        export NVIDIA_VISIBLE_DEVICES=all
        export NVIDIA_DRIVER_CAPABILITIES=all
        ;;
    "amd")
        echo "Setting up for AMD GPU"
        export DOCKER_GPU_ARGS="--device=/dev/dri --device=/dev/kfd"
        export ROC_ENABLE_PRE_VEGA=1
        ;;
    "intel")
        echo "Setting up for Intel GPU"
        export DOCKER_GPU_ARGS="--device=/dev/dri"
        ;;
    *)
        echo "Setting up for generic GPU/software rendering"
        export DOCKER_GPU_ARGS="--device=/dev/dri"
        export LIBGL_ALWAYS_SOFTWARE=1
        ;;
esac

echo "GPU setup complete"