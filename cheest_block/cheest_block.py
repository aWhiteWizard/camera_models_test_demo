import cv2
import numpy as np
def generate_chessboard(cube_cm=2., pattern_size=(6, 8), scale=37.79527559055118):
    """
    generate chessboard image with given cube length, which adapts to A4 paper print
    :param cube_cm: float, single cube length in cm
    :param pattern_size: (x, y), the number of points in x, y axes in the chessboard
    :param scale: float, scale pixel/cm in A4 paper
    """
    # convert cm to pixel
    cube_pixel = cube_cm * scale
    width = round(pattern_size[0] * cube_cm * scale)
    height = round(pattern_size[1] * cube_cm * scale)

    # generate canvas
    image = np.zeros([width, height, 3], dtype=np.uint8)
    image.fill(255)
    color = (255, 255, 255)
    fill_color = 0
    # drawing the chessboard
    for j in range(0, height + 1):
        y = round(j * cube_pixel)
        for i in range(0, width + 1):
            x0 = round(i * cube_pixel)
            y0 = y
            rect_start = (x0, y0)

            x1 = round(x0 + cube_pixel)
            y1 = round(y0 + cube_pixel)
            rect_end = (x1, y1)
            cv2.rectangle(image, rect_start, rect_end, color, 1, 0)
            image[y0:y1, x0:x1] = fill_color
            if width % 2:
                if i != width:
                    fill_color = (0 if (fill_color == 255) else 255)
            else:
                if i != width + 1:
                    fill_color = (0 if (fill_color == 255) else 255)

    # add border around the chess
    chessboard = cv2.copyMakeBorder(image, 30, 30, 30, 30, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255))
    # visualize
    win_name = "chessboard"
    cv2.imshow(win_name, chessboard)
    cv2.waitKey(0)

generate_chessboard()
