#!/bin/bash

# Add shortcut from source to destination
add_shortcut() {
    SOURCE=$1
    TARGET=$2

    if [ ! -f $SOURCE ]; then
        echo "Error: Failed to find source: $SOURCE"
        return
    fi

    TARGET_DIR=$(dirname $TARGET)
    if [ ! -d $TARGET_DIR ]; then
        echo "Error: Destination directory doesn't exist: $TARGET_DIR"
        return
    fi

    if [ -f $TARGET ]; then
        echo "Warning: Installation cancelled, app already exists: $TARGET"
        echo
        return
    fi

    cp $SOURCE $TARGET
    chmod +x $TARGET

    # TODO
}



echo "Creating Calibration Tool desktop launcher shortcur..."

if [[ $EUID -ne 0 ]]; then # root
    LAUNCHER_DESTINATION=/usr/share/applications
else
    LAUNCHER_DESTINATION=~/.local/share/applications
fi

mkdir -p $LAUNCHER_DESTINATION

SOURCE=./CalibrationTool.desktop
TARGET=$LAUNCHER_DESTINATION/CalibrationTool.desktop

add_shortcut $SOURCE $TARGET