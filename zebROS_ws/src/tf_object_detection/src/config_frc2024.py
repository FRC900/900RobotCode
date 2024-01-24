from object_classes import ObjectClasses

# detection model classes
OBJECT_CLASSES = ObjectClasses('/home/ubuntu/2023RobotCode/zebROS_ws/src/tf_object_detection/src/FRC2024.yaml')

# Constants
ALPHA = 0.7
TEXT_SCALE = 1.0
TEXT_THICKNESS = 2
LINE_THICKNESS = 2
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

def _gen_colors(num_colors):
    """Generate different colors.
    # Arguments
      num_colors: total number of colors/classes.
    # Output
      bgrs: a list of (B, G, R) tuples which correspond to each of
            the colors/classes.
    """
    import random
    import colorsys

    hsvs = [[float(x) / num_colors, 1., 0.7] for x in range(num_colors)]
    random.seed(900) # very important
    random.shuffle(hsvs)
    rgbs = list(map(lambda x: list(colorsys.hsv_to_rgb(*x)), hsvs))
    bgrs = [(int(rgb[2] * 255), int(rgb[1] * 255),  int(rgb[0] * 255))
            for rgb in rgbs]
    return bgrs

# colors for per classes
COLORS = _gen_colors(len(OBJECT_CLASSES))