#!/bin/bash

# Remove shortcur from source
remove_shortcut() {
    TARGET=$1

    if [ -f $TARGET ]; then
        rm -f $TARGET
        echo "$TARGET removed."
        echo
    else
        echo "$TARGET already been removed."
        echo
    fi
}

echo "Uninstall Caliration Tool..."

echo "Removing launcher shortcut..."

if [[ $EUID -ne 0 ]]; then # root
    LAUNCHER_DESTINATION=/usr/share/applications
else
    LAUNCHER_DESTINATION=~/.local/share/applications
fi

TARGET=$LAUNCHER_DESTINATION/CalibrationTool.desktop

remove_shortcut $TARGET

echo "Removing desktop shortcut..."

DESTINATION=$(xdg-user-dir DESKTOP)
TARGET=$DESTINATION/CalirationTool.desktop

remove_shortcut $TARGET