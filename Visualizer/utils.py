from abc import ABC, abstractmethod
import cv2


class Section(ABC):
    def __init__(self, name, window):
        self.name = name
        self.window = window
    
    @abstractmethod
    def draw(self):
        pass

def resize_with_aspect_ratio(image, width=None, height=None, inter=cv2.INTER_AREA):
    """
    Resize image while maintaining aspect ratio
    
    Args:
        image: Input image as numpy array
        width: Target width (if None, height will be used)
        height: Target height (if None, width will be used)
        inter: Interpolation method
    
    Returns:
        Resized image
    """
    dim = None
    (h, w) = image.shape[:2]
    
    if width is None and height is None:
        return image
    
    if width is None:
        # Calculate width based on height
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        # Calculate height based on width
        r = width / float(w)
        dim = (width, int(h * r))
    
    return cv2.resize(image, dim, interpolation=inter), dim[0], dim[1]