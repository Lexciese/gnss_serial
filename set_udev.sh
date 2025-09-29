#!/bin/bash
# set_udev.sh - Install GNSS udev rule

RULE_FILE="./99-gnss.rules"
TARGET_DIR="/etc/udev/rules.d"
TARGET_FILE="$TARGET_DIR/99-gnss.rules"

# Check if run as root
if [ "$EUID" -ne 0 ]; then
  echo "❌ Please run as root: sudo ./set_udev.sh"
  exit 1
fi

# Copy rule file
if [ -f "$RULE_FILE" ]; then
  echo "✅ Copying $RULE_FILE to $TARGET_FILE"
  cp "$RULE_FILE" "$TARGET_FILE"
else
  echo "❌ Rule file $RULE_FILE not found!"
  exit 1
fi

# Reload udev
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

echo "✅ Udev rule installed. Replug your device or run 'udevadm trigger'."
