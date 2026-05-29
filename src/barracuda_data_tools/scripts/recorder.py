import os
import subprocess
import datetime
import signal
import sys

TOPICS = [
    #ZED
    "/barracuda/zed_node/rgb/color/rect/image",
    "/barracuda/zed_node/rgb/color/rect/camera_info",
    "/barracuda/zed_node/depth/depth_registered",
    "/barracuda/zed_node/depth/depth_registered/camera_info",
    "/barracuda/zed_node/point_cloud/cloud_registered",
    "/tf",
    "/barracuda/zed_node/odom",
    "/barracuda/zed_node/pose",
    "/barracuda/zed_node/imu/data",

    # dvl
    "/barracuda/dvl/altitude",
    "/barracuda/dvl/odometry",
    "/barracuda/dvl/pose",
    
    # ping360
    # "/barracuda/scan_image",
    # "/barracuda/echo",
    # "/barracuda/scan",

    # thruster manager
    "/barracuda/cmd_thrust",
    "/barracuda/robot_description",
    "/barracuda/wrench",

    # control
    "/barracuda/joy",

    # nvblox
    "/barracuda/nvblox_node/static_map_slice",
    "/barracuda/nvblox_node/static_occupancy_grid",
    "/barracuda/nvblox_node/pessimistic_static_esdf_pointcloud",


]

OUT_DIR = os.path.expanduser("~/barracuda_ws/recordings")
os.makedirs(OUT_DIR, exist_ok=True)

bag_proc = None

def build_bag_cmd(bag_path):
    cmd = ["ros2", "bag", "record", "-o", bag_path]
    cmd.extend(TOPICS)
    return cmd

def check_topics_available():
    """Check if all required topics are being published."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        available_topics = set(result.stdout.strip().split('\n'))
        
        missing_topics = []
        for topic in TOPICS:
            if topic not in available_topics:
                missing_topics.append(topic)
        
        if missing_topics:
            print("[recorder] WARNING: The following topics are NOT being published:")
            for topic in missing_topics:
                print(f"  - {topic}")
            print(f"[recorder] Missing {len(missing_topics)} out of {len(TOPICS)} topics.")
            return False
        else:
            print("[recorder] All required topics are available.")
            return True
    except subprocess.TimeoutExpired:
        print("[recorder] ERROR: Timeout checking topics.")
        return False
    except Exception as e:
        print(f"[recorder] ERROR checking topics: {e}")
        return False

def start_recording():
    global bag_proc
    if bag_proc is not None and bag_proc.poll() is None:
        print("[recorder] Already recording.")
        return
    
    # Check if all topics are available
    if not check_topics_available():
        print("[recorder] Not starting recording due to missing topics.")
        return
    
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_prefix = os.path.join(OUT_DIR, f"zed_{ts}")
    cmd = build_bag_cmd(bag_prefix)
    print("[recorder] Starting:", " ".join(cmd))
    # use Popen so we can terminate later
    log_file = os.path.join(OUT_DIR, f"zed_{ts}.log")
    with open(log_file, 'w') as logf:
        bag_proc = subprocess.Popen(cmd, stdout=logf, stderr=logf, preexec_fn=os.setsid)
    print("[recorder] PID:", bag_proc.pid)

def stop_recording():
    global bag_proc
    if bag_proc is None:
        print("[recorder] Not recording.")
        return
    print("[recorder] Stopping recording PID:", bag_proc.pid)
    try:
        os.killpg(os.getpgid(bag_proc.pid), signal.SIGTERM)
    except Exception as e:
        print("[recorder] Term signal error:", e)
    bag_proc.wait(timeout=5)
    print("[recorder] Recording stopped and saved.")
    bag_proc = None

def main():
    print("Recorder control")
    print("Press 'r' + Enter -> start recording")
    print("Press 's' + Enter -> stop recording")
    print("Press 'q' + Enter -> quit")
    print()
    while True:
        try:
            cmd = input(">> ").strip().lower()
        except EOFError:
            cmd = "q"
        if cmd == "r":
            start_recording()
        elif cmd == "s":
            stop_recording()
            print("[recorder] Ready for next command (r=start, s=stop, q=quit)")
        elif cmd == "q":
            if bag_proc:
                stop_recording()
            print("Bye.")
            break
        else:
            print("Unknown. r=start, s=stop, q=quit.")

if __name__ == "__main__":
    main()