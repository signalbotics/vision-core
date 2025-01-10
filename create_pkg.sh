#!/bin/bash

# Define the root directory
ROOT_DIR=$(pwd)
ARCH=$(dpkg --print-architecture)

# Function to sync models to server
sync_models() {
    echo "Syncing models to server..."
    
    # Create models directory on server if it doesn't exist
    ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
    
    # # Sync YOLO models and source files for detection
    # rsync -a -P "$ROOT_DIR/model/yolov3.weights" root@packages.signalbotics.com:/var/www/packages/models/
    # rsync -a -P "$ROOT_DIR/model/yolov3.cfg" root@packages.signalbotics.com:/var/www/packages/models/
    # rsync -a -P "$ROOT_DIR/model/yolov8s.pt" root@packages.signalbotics.com:/var/www/packages/models/
    # rsync -a -P "$ROOT_DIR/model/yolov8s.onnx" root@packages.signalbotics.com:/var/www/packages/models/
    # rsync -a -P "$ROOT_DIR/model/coco_classes.txt" root@packages.signalbotics.com:/var/www/packages/models/
    # rsync -a -P "$ROOT_DIR/model/coco_colors.txt" root@packages.signalbotics.com:/var/www/packages/models/
    
    # Sync depth models and source files
    rsync -a -P "$ROOT_DIR/models/light_mono.onnx" root@packages.signalbotics.com:/var/www/packages/models/
    rsync -a -P "$ROOT_DIR/models/medium_mono.onnx" root@packages.signalbotics.com:/var/www/packages/models/
    rsync -a -P "$ROOT_DIR/models/light_480x640.onnx" root@packages.signalbotics.com:/var/www/packages/models/
    
    # Sync segmentation model and source files
    # rsync -a -P "$ROOT_DIR/models/segment.onnx" root@packages.signalbotics.com:/var/www/packages/models/
    
    # Sync model modules
    # rsync -a -P "$ROOT_DIR/model" root@packages.signalbotics.com:/var/www/packages/
    # rsync -a -P "$ROOT_DIR/models" root@packages.signalbotics.com:/var/www/packages/
    
    echo "Models synced successfully"
}

# Function to upload package to server
upload_package() {
    local PACKAGE_NAME=$1
    local PACKAGE_FILE="vision_core_${PACKAGE_NAME}_${ARCH}.deb"
    
    # Upload package to server
    rsync -a -P "$PACKAGE_FILE" root@packages.signalbotics.com:/var/www/packages
    ssh root@packages.signalbotics.com "cd /var/www/packages && \
        reprepro remove jammy vision-core-${PACKAGE_NAME} && \
        reprepro includedeb jammy $PACKAGE_FILE"
}

# Function to create common package
create_common_package() {
    local STAGE_DIR="$ROOT_DIR/vision_core_common"
    
    echo "Creating common package..."
    
    # Create staging directory
    if [ -d "$STAGE_DIR" ]; then
        rm -rf "$STAGE_DIR"
    fi
    mkdir -p "$STAGE_DIR"
    mkdir -p "$STAGE_DIR/usr/local/vision_core/cmake" \
            "$STAGE_DIR/usr/local/vision_core/common"
    
    # Copy common files
    cp -r "$ROOT_DIR/cmake/." "$STAGE_DIR/usr/local/vision_core/cmake/"
    cp -r "$ROOT_DIR/common/." "$STAGE_DIR/usr/local/vision_core/common/"
    
    # Copy DEBIAN control file and postinst script
    mkdir -p "$STAGE_DIR/DEBIAN"
    cp "$ROOT_DIR/DEBIAN/control-common" "$STAGE_DIR/DEBIAN/control"
    cp "$ROOT_DIR/DEBIAN/postinst-common" "$STAGE_DIR/DEBIAN/postinst"
    chmod 755 "$STAGE_DIR/DEBIAN/"*
    
    # Build the package
    dpkg-deb --build "$STAGE_DIR" "vision_core_common_${ARCH}.deb"
    
    # Upload to server
    upload_package "common"
    
    # Clean up
    rm -rf "$STAGE_DIR"
}

# Function to create a package
create_package() {
    local MODULE=$1
    local CONTROL_FILE=$2
    local BUILD_OPTION=$3
    local STAGE_DIR="$ROOT_DIR/vision_core_$MODULE"
    
    echo "Creating package for $MODULE module..."
    
    # Create build directory if it doesn't exist
    if [ ! -d "$ROOT_DIR/build_$MODULE" ]; then
        mkdir "$ROOT_DIR/build_$MODULE"
    fi
    
    # Build the module
    cd "$ROOT_DIR/build_$MODULE"
    cmake .. -D$BUILD_OPTION=ON
    make -j$(nproc)
    cd "$ROOT_DIR"
    
    # Create staging directory
    if [ -d "$STAGE_DIR" ]; then
        rm -rf "$STAGE_DIR"
    fi
    mkdir -p "$STAGE_DIR"
    mkdir -p "$STAGE_DIR/usr/local/vision_core/bin" \
            "$STAGE_DIR/usr/local/vision_core/lib" \
            "$STAGE_DIR/usr/local/vision_core/include/vision_core" \
            "$STAGE_DIR/usr/local/vision_core/examples" \
            "$STAGE_DIR/usr/local/vision_core/models"
    
    # Copy module-specific files
    case $MODULE in
        "detect")
            cp -r "$ROOT_DIR/detect/include/." "$STAGE_DIR/usr/local/vision_core/include/vision_core/"
            cp -r "$ROOT_DIR/detect/lib/." "$STAGE_DIR/usr/local/vision_core/lib/"
            cp -r "$ROOT_DIR/examples/detect" "$STAGE_DIR/usr/local/vision_core/examples/"
            ;;
        "segment")
            cp -r "$ROOT_DIR/segment/include/." "$STAGE_DIR/usr/local/vision_core/include/vision_core/"
            cp -r "$ROOT_DIR/segment/lib/." "$STAGE_DIR/usr/local/vision_core/lib/"
            cp -r "$ROOT_DIR/examples/segment" "$STAGE_DIR/usr/local/vision_core/examples/"
            ;;
        "depth")
            cp -r "$ROOT_DIR/mono/include/." "$STAGE_DIR/usr/local/vision_core/include/vision_core/"
            cp -r "$ROOT_DIR/stereo/include/." "$STAGE_DIR/usr/local/vision_core/include/vision_core/"
            cp -r "$ROOT_DIR/mono/lib/." "$STAGE_DIR/usr/local/vision_core/lib/"
            cp -r "$ROOT_DIR/stereo/lib/." "$STAGE_DIR/usr/local/vision_core/lib/"
            cp -r "$ROOT_DIR/examples/mono" "$STAGE_DIR/usr/local/vision_core/examples/"
            cp -r "$ROOT_DIR/examples/stereo" "$STAGE_DIR/usr/local/vision_core/examples/"
            ;;
    esac
    
    # Copy DEBIAN control file and module-specific postinst script
    mkdir -p "$STAGE_DIR/DEBIAN"
    cp "$ROOT_DIR/DEBIAN/$CONTROL_FILE" "$STAGE_DIR/DEBIAN/control"
    if [ "$MODULE" = "common" ]; then
        cp "$ROOT_DIR/DEBIAN/postinst-common" "$STAGE_DIR/DEBIAN/postinst"
    else
        cp "$ROOT_DIR/DEBIAN/postinst-$MODULE" "$STAGE_DIR/DEBIAN/postinst"
    fi
    chmod 755 "$STAGE_DIR/DEBIAN/"*
    
    # Build the package
    dpkg-deb --build "$STAGE_DIR" "vision_core_${MODULE}_${ARCH}.deb"
    
    # Upload to server
    upload_package "$MODULE"
    
    # Clean up
    rm -rf "$STAGE_DIR"
    rm -rf "$ROOT_DIR/build_$MODULE"
}

# Create meta package
create_meta_package() {
    local STAGE_DIR="$ROOT_DIR/vision_core_meta"
    
    if [ -d "$STAGE_DIR" ]; then
        rm -rf "$STAGE_DIR"
    fi
    mkdir -p "$STAGE_DIR/DEBIAN"
    
    # Copy control file
    cp "$ROOT_DIR/DEBIAN/control-meta" "$STAGE_DIR/DEBIAN/control"
    chmod 755 "$STAGE_DIR/DEBIAN/"*
    
    # Build the package
    dpkg-deb --build "$STAGE_DIR" "vision_core_${ARCH}.deb"
    
    # Upload meta package
    mv "vision_core_${ARCH}.deb" "vision_core_meta_${ARCH}.deb"
    upload_package "meta"
    
    # Clean up
    rm -rf "$STAGE_DIR"
}

# Parse command line arguments
MODULE=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --common)
            create_common_package
            echo "Installing common package..."
            sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            shift
            ;;
        --detect)
            # Sync detection models
            echo "Syncing detection models..."
            ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
            # rsync -a -P "$ROOT_DIR/model/yolov3.weights" root@packages.signalbotics.com:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/model/yolov3.cfg" root@packages.signalbotics.com:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/model/yolov8s.pt" root@64.52.108.220:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/model/coco_classes.txt" root@64.52.108.220:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/model/coco_colors.txt" root@64.52.108.220:/var/www/packages/models/
            
            # Ensure common package is installed first
            if ! dpkg -l | grep -q "^ii.*vision-core-common"; then
                create_common_package
                sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            fi
            create_package "detect" "control-detect" "BUILD_DETECT"
            sudo dpkg -i vision_core_detect_${ARCH}.deb || exit 1
            shift
            ;;
        --segment)
            # Sync segmentation model
            echo "Syncing segmentation model..."
            ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
            # rsync -a -P "$ROOT_DIR/models/segment.onnx" root@64.52.108.220:/var/www/packages/models/
            
            # Ensure common package is installed first
            if ! dpkg -l | grep -q "^ii.*vision-core-common"; then
                create_common_package
                sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            fi
            create_package "segment" "control-segment" "BUILD_SEGMENT"
            sudo dpkg -i vision_core_segment_${ARCH}.deb || exit 1
            shift
            ;;
        --depth)
            # Sync depth models
            echo "Syncing depth models..."
            ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
            rsync -a -P "$ROOT_DIR/models/light_mono.onnx" root@64.52.108.220:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/models/medium_mono.onnx" root@64.52.108.220:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/models/light_480x640.onnx" root@64.52.108.220:/var/www/packages/models/
            
            # Ensure common package is installed first
            if ! dpkg -l | grep -q "^ii.*vision-core-common"; then
                create_common_package
                sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            fi
            create_package "depth" "control-depth" "BUILD_DEPTH"
            sudo dpkg -i vision_core_depth_${ARCH}.deb || exit 1
            shift
            ;;
        --all)
            # Sync models first
            sync_models
            
            # Build packages
            create_common_package
            create_package "detect" "control-detect" "BUILD_DETECT"
            create_package "segment" "control-segment" "BUILD_SEGMENT"
            create_package "depth" "control-depth" "BUILD_DEPTH"
            create_meta_package
            
            # Install packages in correct order
            echo "Installing packages..."
            sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_detect_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_segment_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_depth_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_meta_${ARCH}.deb || exit 1
            echo "All packages installed successfully!"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--common] [--detect] [--segment] [--depth] [--all] [--sync-models]"
            exit 1
            ;;
        --sync-models)
            sync_models
            shift
            ;;
    esac
done

if [ $# -eq 0 ]; then
    echo "Usage: $0 [--common] [--detect] [--segment] [--depth] [--all] [--sync-models]"
    echo "  --common  : Build common package"
    echo "  --detect  : Build detection module package"
    echo "  --segment : Build segmentation module package"
    echo "  --depth   : Build depth estimation module package"
    echo "  --all     : Build all modules and meta package"
    exit 1
fi
