#!/bin/bash
# MeshLib WASM Build Script for WSL
# This script sets up Emscripten and builds MeshLib for WebAssembly

set -e

echo "=============================================="
echo "MeshLib WASM Build Script"
echo "=============================================="

# Configuration
MESHLIB_DIR="/mnt/c/Users/VijayRaghavVarada/Documents/Github/MeshLib"
EMSDK_DIR="$HOME/emsdk"

# Step 1: Install Emscripten SDK
install_emsdk() {
    echo ""
    echo "[Step 1] Installing Emscripten SDK..."
    
    if [ -d "$EMSDK_DIR" ]; then
        echo "Emscripten SDK already exists at $EMSDK_DIR"
        cd "$EMSDK_DIR"
        git pull
    else
        echo "Cloning Emscripten SDK..."
        git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_DIR"
        cd "$EMSDK_DIR"
    fi
    
    echo "Installing latest Emscripten..."
    ./emsdk install latest
    ./emsdk activate latest
    source ./emsdk_env.sh
    
    echo "Emscripten version:"
    emcc --version
}

# Step 2: Build third-party dependencies
build_thirdparty() {
    echo ""
    echo "[Step 2] Building third-party dependencies with Emscripten..."
    
    cd "$MESHLIB_DIR"
    
    # Source Emscripten environment
    source "$EMSDK_DIR/emsdk_env.sh"
    
    # Set environment variables
    export MR_EMSCRIPTEN=ON
    export MR_EMSCRIPTEN_SINGLE=OFF  # Multi-threaded for better performance
    
    # Build thirdparty
    ./scripts/build_thirdparty.sh
}

# Step 3: Build MeshLib WASM
build_meshlib() {
    echo ""
    echo "[Step 3] Building MeshLib for WebAssembly..."
    
    cd "$MESHLIB_DIR"
    source "$EMSDK_DIR/emsdk_env.sh"
    
    # Configure CMake
    cmake --preset wasm32-mt-release
    
    # Build
    cmake --build ./build/wasm32-mt-release --parallel
    
    echo ""
    echo "Build complete! Output files:"
    ls -la ./build/wasm32-mt-release/bin/
}

# Main execution
main() {
    install_emsdk
    build_thirdparty
    build_meshlib
    
    echo ""
    echo "=============================================="
    echo "MeshLib WASM build completed successfully!"
    echo "=============================================="
}

# Run main function
main