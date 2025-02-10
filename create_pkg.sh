#!/bin/bash

# Define the root directory
ROOT_DIR=$(pwd)
ARCH=$(dpkg --print-architecture)

# Function to sync models to server
sync_models() {
    echo "Syncing models to server..."
    
    # Create models directory on server if it doesn't exist
    ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
    
    # Read current version from version file or create if doesn't exist
    VERSION_FILE="$ROOT_DIR/models/version.txt"
    if [ ! -f "$VERSION_FILE" ]; then
        echo "1.0.0" > "$VERSION_FILE"
    fi
    CURRENT_VERSION=$(cat "$VERSION_FILE")
    
    # Increment patch version
    MAJOR=$(echo "$CURRENT_VERSION" | cut -d. -f1)
    MINOR=$(echo "$CURRENT_VERSION" | cut -d. -f2)
    PATCH=$(echo "$CURRENT_VERSION" | cut -d. -f3)
    PATCH=$((PATCH + 1))
    NEW_VERSION="$MAJOR.$MINOR.$PATCH"
    echo "$NEW_VERSION" > "$VERSION_FILE"
    
    # Sync models based on type
    case $1 in
        "detect")
            echo "Uploading detection models..."
            rsync -a -P "$ROOT_DIR/model/yolov8s.onnx" root@packages.signalbotics.com:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/model/coco_classes.txt" root@packages.signalbotics.com:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/model/coco_colors.txt" root@packages.signalbotics.com:/var/www/packages/models/
            rsync -a -P "$VERSION_FILE" root@packages.signalbotics.com:/var/www/packages/models/
            ;;
        "depth")
            echo "Uploading depth models..."
            rsync -a -P "$ROOT_DIR/models/light_mono.onnx" root@packages.signalbotics.com:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/models/medium_mono.onnx" root@packages.signalbotics.com:/var/www/packages/models/
            rsync -a -P "$ROOT_DIR/models/light_480x640.onnx" root@packages.signalbotics.com:/var/www/packages/models/
            ;;
        "segment")
            echo "Uploading segmentation models..."
            rsync -a -P "$ROOT_DIR/models/segment.onnx" root@packages.signalbotics.com:/var/www/packages/models/
            ;;
        *)
            ;;
    esac
    
    echo "Models synced successfully"
}

# Function to upload package to server
upload_package() {
    local PACKAGE_NAME=$1
    local PACKAGE_FILE="vision_core_${PACKAGE_NAME}_${ARCH}.deb"
    
    echo "Uploading $PACKAGE_FILE to server..."
    rsync -a -P "$PACKAGE_FILE" root@packages.signalbotics.com:/var/www/packages
    ssh root@packages.signalbotics.com "cd /var/www/packages && \
        reprepro remove jammy vision-core-${PACKAGE_NAME} && \
        reprepro includedeb jammy $PACKAGE_FILE"
    echo "Upload complete for $PACKAGE_FILE"
}

# Function to upload all packages
upload_all_packages() {
    echo "Uploading all packages to server..."
    for pkg in "common" "detect" "segment" "depth" "firmware" "meta"; do
        if [ -f "vision_core_${pkg}_${ARCH}.deb" ]; then
            upload_package "$pkg"
        fi
    done
    echo "All uploads complete"
}

# Function to create common package
create_common_package() {
    local STAGE_DIR="$ROOT_DIR/vision_core_common"
    
    echo "Creating common package..."
    
    # Create staging directory
    if [ -d "$STAGE_DIR" ]; then
        rm -rf "$STAGE_DIR"
    fi
    mkdir -p "$STAGE_DIR/usr/local/vision_core/cmake" \
            "$STAGE_DIR/usr/local/vision_core/common"
    
    # Copy common files
    cp -r "$ROOT_DIR/cmake/." "$STAGE_DIR/usr/local/vision_core/cmake/"
    cp -r "$ROOT_DIR/common/." "$STAGE_DIR/usr/local/vision_core/common/"
    cp -r "$ROOT_DIR/models/$VERSION_FILE" "$STAGE_DIR/usr/local/vision_core/models/"
    
    # Copy DEBIAN control file and postinst script
    mkdir -p "$STAGE_DIR/DEBIAN"
    cp "$ROOT_DIR/DEBIAN/control-common" "$STAGE_DIR/DEBIAN/control"
    cp "$ROOT_DIR/DEBIAN/postinst-common" "$STAGE_DIR/DEBIAN/postinst"
    chmod 755 "$STAGE_DIR/DEBIAN/"*
    
    # Build the package
    dpkg-deb --build "$STAGE_DIR" "vision_core_common_${ARCH}.deb"
    
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
        "firmware")
            cp -r "$ROOT_DIR/firmwares/include/." "$STAGE_DIR/usr/local/vision_core/include/vision_core/"
            cp -r "$ROOT_DIR/firmwares/lib/." "$STAGE_DIR/usr/local/vision_core/lib/"
            cp -r "$ROOT_DIR/examples/nreal" "$STAGE_DIR/usr/local/vision_core/examples/"
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
    dpkg-deb --build "$STAGE_DIR" "vision_core_meta_${ARCH}.deb"

    # Clean up
    rm -rf "$STAGE_DIR"
}

# Parse command line arguments
MODULE=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --upload)
            if [ -z "$2" ]; then
                echo "Please specify what to upload: package, models, or all"
                exit 1
            elif [ "$2" = "all" ]; then
                upload_all_packages
                sync_models
            elif [ "$2" = "models" ]; then
                sync_models
            elif [ "$2" = "packages" ]; then
                upload_all_packages
            else
                upload_package "$2"
            fi
            shift 2
            ;;
        --common)
            create_common_package
            echo "Installing common package..."
            sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            shift
            ;;
        --detect)
            # Sync detection models
            echo "Syncing detection models..."
            # ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
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
            # ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
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
            # ssh root@packages.signalbotics.com "mkdir -p /var/www/packages/models"
            # rsync -a -P "$ROOT_DIR/models/light_mono.onnx" root@64.52.108.220:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/models/medium_mono.onnx" root@64.52.108.220:/var/www/packages/models/
            # rsync -a -P "$ROOT_DIR/models/light_480x640.onnx" root@64.52.108.220:/var/www/packages/models/
            
            # Ensure common package is installed first
            if ! dpkg -l | grep -q "^ii.*vision-core-common"; then
                create_common_package
                sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            fi
            create_package "depth" "control-depth" "BUILD_DEPTH"
            sudo dpkg -i vision_core_depth_${ARCH}.deb || exit 1
            shift
            ;;
        --firmware)
            
            # Ensure common package is installed first
            if ! dpkg -l | grep -q "^ii.*vision-core-common"; then
                create_common_package
                sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            fi
            create_package "firmware" "control-firmware" "BUILD_FIRMWARE"
            sudo dpkg -i vision_core_firmware_${ARCH}.deb || exit 1
            shift
            ;;
        --all)
            
            # Build packages
            create_common_package
            create_package "detect" "control-detect" "BUILD_DETECT"
            create_package "segment" "control-segment" "BUILD_SEGMENT"
            create_package "depth" "control-depth" "BUILD_DEPTH"
            create_package "firmware" "control-firmware" "BUILD_FIRMWARE"
            create_meta_package
            
            # Install packages in correct order
            echo "Installing packages..."
            sudo dpkg -i vision_core_common_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_detect_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_segment_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_firmware_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_depth_${ARCH}.deb || exit 1
            sudo dpkg -i vision_core_meta_${ARCH}.deb || exit 1
            echo "All packages installed successfully!"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--common] [--detect] [--segment] [--depth] [--firmware] [--all] [--upload <package|packages|models|all>]"
            exit 1
            ;;
    esac
done

if [ $# -eq 0 ]; then
    echo "Usage: $0 [--common] [--detect] [--segment] [--depth] [--firmware] [--all] [--upload <package|packages|models|all>]"
    echo "  --common  : Build common package"
    echo "  --detect  : Build detection module package"
    echo "  --segment : Build segmentation module package"
    echo "  --depth   : Build depth estimation module package"
    echo "  --firmware: Build firmware module package"
    echo "  --all     : Build all modules and meta package"
    echo "  --upload: Upload to server:"
    echo "           package  : Upload specific package (e.g., --upload detect)"
    echo "           packages : Upload all packages"
    echo "           models   : Upload all models"
    echo "           all      : Upload both packages and models"
    exit 1
fi
