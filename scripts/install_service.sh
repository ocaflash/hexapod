#!/bin/bash
# Install hexapod as systemd service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/hexapod.service"

echo "=== Installing Hexapod Service ==="

# Make scripts executable
chmod +x "$SCRIPT_DIR/run_hexapod.sh"
chmod +x "$SCRIPT_DIR/update_and_build.sh"
chmod +x "$SCRIPT_DIR/setup_workspace.sh"

# Copy service file
sudo cp "$SERVICE_FILE" /etc/systemd/system/hexapod.service

# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable hexapod.service

echo "=== Service installed ==="
echo ""
echo "Commands:"
echo "  sudo systemctl start hexapod    # Start now"
echo "  sudo systemctl stop hexapod     # Stop"
echo "  sudo systemctl status hexapod   # Check status"
echo "  sudo journalctl -u hexapod -f   # View logs"
echo ""
echo "To disable autostart: sudo systemctl disable hexapod"
