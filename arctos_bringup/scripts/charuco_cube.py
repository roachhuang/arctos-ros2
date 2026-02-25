#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2


def build_board(dictionary_id: int, squares_x: int, squares_y: int, square_length: float, marker_length: float):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(dictionary_id)

    # Prefer CharucoBoard_create + draw for OpenCV 4.6 stability.
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, dictionary)
    return aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)


def render_board(board, width_px: int, height_px: int, margin_px: int, border_bits: int):
    size = (width_px, height_px)
    if hasattr(board, "draw"):
        return board.draw(size, marginSize=margin_px, borderBits=border_bits)
    return board.generateImage(size, marginSize=margin_px, borderBits=border_bits)


def main():
    parser = argparse.ArgumentParser(description="Generate a printable ChArUco board image.")
    parser.add_argument("--squares-x", type=int, default=5, help="Number of chessboard squares in X.")
    parser.add_argument("--squares-y", type=int, default=7, help="Number of chessboard squares in Y.")
    parser.add_argument("--square-length", type=float, default=0.035, help="Square size in meters.")
    parser.add_argument("--marker-length", type=float, default=0.022, help="Marker size in meters.")
    parser.add_argument(
        "--dictionary-id",
        type=int,
        default=int(cv2.aruco.DICT_5X5_250),
        help="OpenCV ArUco dictionary id (e.g. DICT_5X5_250=7).",
    )
    parser.add_argument("--width-px", type=int, default=1200, help="Output image width in pixels.")
    parser.add_argument("--height-px", type=int, default=1600, help="Output image height in pixels.")
    parser.add_argument("--margin-px", type=int, default=20, help="Margin in pixels.")
    parser.add_argument("--border-bits", type=int, default=1, help="Marker border bits.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("charuco_face.png"),
        help="Output image path.",
    )
    args = parser.parse_args()

    if args.marker_length >= args.square_length:
        raise ValueError("marker-length must be smaller than square-length.")

    board = build_board(
        args.dictionary_id,
        args.squares_x,
        args.squares_y,
        args.square_length,
        args.marker_length,
    )
    image = render_board(board, args.width_px, args.height_px, args.margin_px, args.border_bits)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(args.output), image)
    if not ok:
        raise RuntimeError(f"Failed to write output image: {args.output}")

    print(f"Saved: {args.output}")
    print(
        f"Board: {args.squares_x}x{args.squares_y}, square={args.square_length}m, "
        f"marker={args.marker_length}m, dict={args.dictionary_id}"
    )


if __name__ == "__main__":
    main()
