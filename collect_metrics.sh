#!/bin/bash

echo "=== D* Lite Metrics Collection with rqt_console ==="
echo ""
echo "This script will help you collect metrics from your D* Lite planner using rqt_console."
echo ""

# Build the package
echo "1. Building the D* Lite planner..."
cd ../..
colcon build --packages-select nav2_dstar_lite_planner

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build successful!"
echo ""

# Source the workspace
source install/setup.bash

echo "2. Starting rqt_console for metrics collection..."
echo "   - rqt_console will capture all log messages"
echo "   - Look for lines starting with 'METRICS:'"
echo "   - These contain: path_length, planning_time_ms, test_type"
echo ""

# Start rqt_console in background
rqt_console &
RQT_PID=$!

echo "3. rqt_console started (PID: $RQT_PID)"
echo ""
echo "4. Now you can:"
echo "   - Use Nav2 with your D* Lite planner"
echo "   - Send navigation goals"
echo "   - Watch for METRICS messages in rqt_console"
echo ""
echo "5. To collect data:"
echo "   - In rqt_console, filter by 'METRICS'"
echo "   - Export the filtered logs to CSV"
echo "   - Or copy the metrics lines manually"
echo ""
echo "6. Example metrics format:"
echo "   METRICS: INITIAL | path_length=5.234 | planning_time_ms=12.45 | test_type=initial"
echo "   METRICS: REPLANNING | path_length=5.456 | planning_time_ms=3.21 | test_type=replanning"
echo ""
echo "7. To stop:"
echo "   - Press Ctrl+C to stop this script"
echo "   - rqt_console will continue running"
echo ""

# Wait for user to stop
trap "echo 'Stopping metrics collection...'; kill $RQT_PID 2>/dev/null; exit 0" INT

echo "Press Ctrl+C to stop the metrics collection..."
while true; do
    sleep 1
done 