import cv2

print("Scanning for cameras... this may take a minute...")

# Scan indexes 0 through 50
for index in range(50):
    cap = cv2.VideoCapture(index)
    
    # Try to open the camera
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"✅ SUCCESS: Camera found at Index {index}!")
            print(f"   Resolution: {int(cap.get(3))}x{int(cap.get(4))}")
            # Release it immediately so we don't lock it
            cap.release()
        else:
            print(f"❌ Index {index} opens, but returns empty frames (likely Pi Camera internal node).")
    cap.release()

print("Scan complete.")
