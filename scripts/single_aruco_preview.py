#!/usr/bin/env python3
"""RGB-only ArUco preview for one V4L2 camera, streamed as raw BGR."""

import argparse
import sys
import time

import cv2


EXPECTED_IDS = frozenset((2, 3, 4))
MARKER_SIZE_MM = 30


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video8")
    parser.add_argument("--serial", default="201623028735")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--display-scale", type=float, default=0.5)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), parameters
    )

    frame_count = 0
    started = time.monotonic()
    last_report = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("RGB frame acquisition failed")

            corners, ids, _ = detector.detectMarkers(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            )
            marker_ids = () if ids is None else tuple(
                sorted(int(value) for value in ids.flatten())
            )
            if ids is not None:
                expected = [
                    index for index, value in enumerate(ids.flatten())
                    if int(value) in EXPECTED_IDS
                ]
                other = [
                    index for index, value in enumerate(ids.flatten())
                    if int(value) not in EXPECTED_IDS
                ]
                if expected:
                    cv2.aruco.drawDetectedMarkers(
                        frame, [corners[index] for index in expected],
                        ids[expected], (0, 255, 0)
                    )
                if other:
                    cv2.aruco.drawDetectedMarkers(
                        frame, [corners[index] for index in other],
                        ids[other], (255, 0, 255)
                    )

            frame_count += 1
            rate = frame_count / max(time.monotonic() - started, 1e-6)
            arm_ids = tuple(value for value in marker_ids if value in EXPECTED_IDS)
            other_ids = tuple(value for value in marker_ids if value not in EXPECTED_IDS)
            missing = tuple(sorted(EXPECTED_IDS.difference(arm_ids)))
            cv2.putText(
                frame, f"{args.serial} | arm {arm_ids or '-'} | other {other_ids or '-'}",
                (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2,
                cv2.LINE_AA
            )
            cv2.putText(
                frame, f"missing {missing or '-'} | {MARKER_SIZE_MM} mm | {rate:.1f} Hz",
                (25, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                cv2.LINE_AA
            )

            if marker_ids != last_report:
                print(f"camera_ids={marker_ids}", file=sys.stderr, flush=True)
                last_report = marker_ids

            output = cv2.resize(
                frame, None, fx=args.display_scale, fy=args.display_scale,
                interpolation=cv2.INTER_AREA
            )
            try:
                sys.stdout.buffer.write(output.tobytes())
                sys.stdout.buffer.flush()
            except BrokenPipeError:
                break
    finally:
        capture.release()


if __name__ == "__main__":
    main()
