from localization import Localization
import argparse
import cv2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parameters to run camera")
    parser.add_argument('--camera', type=str, default='camera4')
    args = parser.parse_args()
    local = Localization(camera=args.camera)
    while True:
        local.localize()






