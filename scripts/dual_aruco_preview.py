#!/usr/bin/env python3
"""Live RGB-only ArUco preview for the two directly connected RealSense D435s."""

import argparse
import sys
import time

import cv2


CAMERAS = (
    ("814113021800", "/dev/video8"),
    ("201623028735", "/dev/video14"),
)

EXPECTED_IDS = frozenset((2, 3, 4))
MARKER_SIZE_M = 0.030

DICTIONARIES = (
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_5X5_50", cv2.aruco.DICT_5X5_50),
    ("DICT_6X6_50", cv2.aruco.DICT_6X6_50),
    ("DICT_7X7_50", cv2.aruco.DICT_7X7_50),
    ("DICT_ARUCO_ORIGINAL", cv2.aruco.DICT_ARUCO_ORIGINAL),
)


def open_camera(node: str, width: int, height: int, fps: int):
    capture = cv2.VideoCapture(node, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {node}")
    actual = (
        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        capture.get(cv2.CAP_PROP_FPS),
    )
    if actual != (width, height, float(fps)):
        raise RuntimeError(f"{node}: requested {(width, height, fps)}, got {actual}")
    return capture


def make_detectors():
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return [
        (name, cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(identifier), parameters))
        for name, identifier in DICTIONARIES
    ]


def detect(gray, detectors, locked_name):
    candidates = detectors if locked_name is None else (
        item for item in detectors if item[0] == locked_name)
    for name, detector in candidates:
        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is not None and len(ids):
            return name, corners, ids, rejected
    return locked_name, (), None, ()


def keep_expected(corners, ids):
    if ids is None:
        return (), None
    selected = [
        index for index, value in enumerate(ids.flatten())
        if int(value) in EXPECTED_IDS
    ]
    if not selected:
        return (), None
    return [corners[index] for index in selected], ids[selected]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--display-scale", type=float, default=0.5)
    args = parser.parse_args()

    detectors = make_detectors()
    captures = [
        open_camera(node, args.width, args.height, args.fps)
        for _, node in CAMERAS
    ]
    locked_dictionary = "DICT_4X4_50"
    last_report = None
    frame_count = 0
    start = time.monotonic()
    try:
        while True:
            frames = []
            for capture in captures:
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("RGB frame acquisition failed")
                frames.append(frame)

            results = []
            for frame in frames:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                name, corners, ids, _ = detect(
                    gray, detectors, locked_dictionary)
                results.append((corners, ids))

            # If one view found the dictionary, redetect the other with it.
            if locked_dictionary is not None:
                for index, (corners, ids) in enumerate(results):
                    if ids is None:
                        gray = cv2.cvtColor(frames[index], cv2.COLOR_BGR2GRAY)
                        _, corners, ids, _ = detect(
                            gray, detectors, locked_dictionary)
                        results[index] = (corners, ids)

            frame_count += 1
            elapsed = max(time.monotonic() - start, 1e-6)
            processing_fps = frame_count / elapsed
            report = []
            for index, (frame, result) in enumerate(zip(frames, results)):
                corners, ids = result
                if ids is not None:
                    marker_ids = tuple(sorted(int(value) for value in ids.flatten()))
                    expected_indices = [
                        item for item, value in enumerate(ids.flatten())
                        if int(value) in EXPECTED_IDS
                    ]
                    other_indices = [
                        item for item, value in enumerate(ids.flatten())
                        if int(value) not in EXPECTED_IDS
                    ]
                    if expected_indices:
                        cv2.aruco.drawDetectedMarkers(
                            frame,
                            [corners[item] for item in expected_indices],
                            ids[expected_indices], (0, 255, 0),
                        )
                    if other_indices:
                        cv2.aruco.drawDetectedMarkers(
                            frame,
                            [corners[item] for item in other_indices],
                            ids[other_indices], (255, 0, 255),
                        )
                else:
                    marker_ids = ()
                expected_found = tuple(
                    value for value in marker_ids if value in EXPECTED_IDS)
                other_found = tuple(
                    value for value in marker_ids if value not in EXPECTED_IDS)
                report.append(marker_ids)
                missing_ids = tuple(sorted(EXPECTED_IDS.difference(expected_found)))
                label = locked_dictionary or "scanning dictionaries"
                cv2.putText(
                    frame,
                    f"{CAMERAS[index][0]} | arm {expected_found or '-'} | "
                    f"other/ref? {other_found or '-'}",
                    (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"missing {missing_ids or '-'} | size {MARKER_SIZE_M*1000:.0f} mm | "
                    f"loop {processing_fps:.1f} Hz",
                    (25, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                    cv2.LINE_AA,
                )
                frames[index] = cv2.resize(
                    frame, None,
                    fx=args.display_scale, fy=args.display_scale,
                    interpolation=cv2.INTER_AREA,
                )

            current_report = (locked_dictionary, tuple(report))
            if current_report != last_report:
                print(
                    f"dictionary={locked_dictionary or 'scanning'} "
                    f"camera_ids={report}",
                    file=sys.stderr, flush=True,
                )
                last_report = current_report
            mosaic = cv2.hconcat(frames)
            try:
                sys.stdout.buffer.write(mosaic.tobytes())
                sys.stdout.buffer.flush()
            except BrokenPipeError:
                break
    finally:
        for capture in captures:
            capture.release()


if __name__ == "__main__":
    main()
