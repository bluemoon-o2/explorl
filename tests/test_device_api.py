import sys
import os

# Add the lib path to sys.path
lib_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "explorl/lib"))
if lib_path not in sys.path:
    sys.path.append(lib_path)

try:
    import explorl_core
except ImportError:
    # Try Release subfolder if plain lib fails (based on build log)
    lib_path_release = os.path.join(lib_path, "Release")
    if lib_path_release not in sys.path:
        sys.path.append(lib_path_release)
    try:
        import explorl_core
    except ImportError as e:
        print(f"Failed to import explorl_core: {e}")
        sys.exit(1)

print("--- Testing Device API ---")

# Check available devices
try:
    devices = explorl_core.render.get_available_devices()
    print(f"Available Devices ({len(devices)}):")
    for i, dev in enumerate(devices):
        print(f"  [{i}] {dev.name} (Memory: {dev.memory_mb} MB, Discrete: {dev.is_discrete})")
except Exception as e:
    print(f"Error getting devices: {e}")

# Check current device (should be unknown/no context)
try:
    current = explorl_core.render.get_current_device_name()
    print(f"Current Device (Pre-Window): {current}")
except Exception as e:
    print(f"Error getting current device: {e}")
