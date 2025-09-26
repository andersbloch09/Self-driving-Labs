#!/bin/bash

# DevContainer GPU Configuration Selector
# This script helps select the appropriate devcontainer configuration based on your GPU

echo "=== DevContainer GPU Configuration Selector ==="
echo

# Function to detect GPU type
detect_gpu() {
    local gpu_type="unknown"
    local gpu_info=""
    
    # Check for NVIDIA GPU
    if command -v nvidia-smi >/dev/null 2>&1; then
        if nvidia-smi >/dev/null 2>&1; then
            gpu_type="nvidia"
            gpu_info=$(nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -1)
            echo "✓ NVIDIA GPU detected: $gpu_info"
        fi
    fi
    
    # Check for AMD GPU
    if lspci | grep -i amd | grep -i vga >/dev/null 2>&1; then
        local amd_info=$(lspci | grep -i amd | grep -i vga | head -1 | sed 's/.*: //')
        if [ "$gpu_type" = "nvidia" ]; then
            echo "✓ AMD GPU also detected: $amd_info (hybrid system)"
            gpu_type="hybrid"
        else
            gpu_type="amd"
            echo "✓ AMD GPU detected: $amd_info"
        fi
    fi
    
    # Check for Intel GPU
    if lspci | grep -i intel | grep -i graphics >/dev/null 2>&1; then
        local intel_info=$(lspci | grep -i intel | grep -i graphics | head -1 | sed 's/.*: //')
        if [ "$gpu_type" = "unknown" ]; then
            gpu_type="intel"
            echo "✓ Intel GPU detected: $intel_info"
        else
            echo "✓ Intel GPU also detected: $intel_info"
        fi
    fi
    
    if [ "$gpu_type" = "unknown" ]; then
        echo "⚠ No dedicated GPU detected"
    fi
    
    echo "$gpu_type"
}

# Main logic
echo "Detecting GPU hardware..."
GPU_DETECTION_OUTPUT=$(detect_gpu)
GPU_TYPE=$(echo "$GPU_DETECTION_OUTPUT" | tail -1)

echo "$GPU_DETECTION_OUTPUT" | head -n -1
echo
echo "GPU Detection Result: $GPU_TYPE"
echo

case $GPU_TYPE in
    "nvidia")
        echo "🎯 RECOMMENDED: Use devcontainer.nvidia.json for best performance"
        echo "   This provides CUDA support and optimal NVIDIA GPU acceleration"
        echo
        echo "To use this configuration:"
        echo "1. Copy devcontainer.nvidia.json to devcontainer.json:"
        echo "   cp .devcontainer/devcontainer.nvidia.json .devcontainer/devcontainer.json"
        echo "2. Restart your devcontainer"
        ;;
    "amd")
        echo "🎯 RECOMMENDED: Use devcontainer.amd.json for best performance"
        echo "   This provides ROCm support and optimal AMD GPU acceleration"
        echo
        echo "To use this configuration:"
        echo "1. Copy devcontainer.amd.json to devcontainer.json:"
        echo "   cp .devcontainer/devcontainer.amd.json .devcontainer/devcontainer.json"
        echo "2. Restart your devcontainer"
        ;;
    "hybrid")
        echo "🎯 DETECTED: Multiple GPUs (NVIDIA + AMD)"
        echo "   You can choose either configuration based on your preferred GPU:"
        echo "   - For NVIDIA: cp .devcontainer/devcontainer.nvidia.json .devcontainer/devcontainer.json"
        echo "   - For AMD:    cp .devcontainer/devcontainer.amd.json .devcontainer/devcontainer.json"
        ;;
    "intel"|"unknown")
        echo "🎯 RECOMMENDED: Use default devcontainer.json (Universal)"
        echo "   This provides software rendering with generic GPU support"
        echo "   Current configuration should work as-is."
        ;;
esac

echo
echo "📋 Alternative: The default devcontainer.json provides universal compatibility"
echo "   but may not offer optimal performance for your specific GPU."
echo
echo "💡 After changing configuration, rebuild your container with:"
echo "   'Dev Containers: Rebuild Container' in VS Code"