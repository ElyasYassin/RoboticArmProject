import numpy as np
import cv2

# --- Initialize Color Thresholds and Kernel ---
green_lower = np.array([35, 100, 50], np.uint8)
green_upper = np.array([85, 255, 255], np.uint8)

purple_lower = np.array([135, 100, 50], np.uint8)
purple_upper = np.array([155, 255, 255], np.uint8)

kernel = np.ones((5, 5), "uint8")

def detect_color_from_top_half_once(frame, min_pixel_threshold=100):
    """
    Single frame color detection (green, purple, or unknown).
    """
    h, w, _ = frame.shape

    # Only work on top half
    top_half = frame[0:h//2, :]

    # Convert to HSV
    hsv_top = cv2.cvtColor(top_half, cv2.COLOR_BGR2HSV)

    # Create color masks
    green_mask = cv2.inRange(hsv_top, green_lower, green_upper)
    purple_mask = cv2.inRange(hsv_top, purple_lower, purple_upper)

    # Morphological transformations
    green_mask = cv2.dilate(green_mask, kernel)
    purple_mask = cv2.dilate(purple_mask, kernel)

    # Count pixels
    green_pixels = cv2.countNonZero(green_mask)
    purple_pixels = cv2.countNonZero(purple_mask)

    detected_color = "unknown"
    if green_pixels > purple_pixels and green_pixels > min_pixel_threshold:
        detected_color = "green"
    elif purple_pixels > green_pixels and purple_pixels > min_pixel_threshold:
        detected_color = "purple"

    return detected_color

def detect_color(camera_index=1, max_attempts=10, min_pixel_threshold=100, visualize=False):
    """
    Repeatedly captures frames and tries to detect color up to max_attempts.

    Args:
        camera_index (int): Webcam index (default 0).
        max_attempts (int): How many frames to try before giving up.
        min_pixel_threshold (int): Minimum pixel count to accept detection.
        visualize (bool): If True, shows debug window.

    Returns:
        detected_color (str): 'green', 'purple', or 'unknown'
    """
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Camera not accessible!")
        return "unknown"

    detected_color = "unknown"

    for attempt in range(max_attempts):
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            continue

        detected_color = detect_color_from_top_half_once(frame, min_pixel_threshold=min_pixel_threshold)

        if visualize:
            annotated_frame = frame.copy()
            h, w, _ = annotated_frame.shape
            cv2.putText(annotated_frame, f"Detected: {detected_color}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.line(annotated_frame, (0, h//2), (w, h//2), (0, 255, 255), 2)
            cv2.imshow("Color Detection Debug", annotated_frame)
            cv2.waitKey(1)

        if detected_color != "unknown":
            print(f"Detected {detected_color} after {attempt+1} attempts.")
            break

    cap.release()
    cv2.destroyAllWindows()

    if detected_color == "unknown":
        print(f"Failed to detect color after {max_attempts} attempts.")

    return detected_color