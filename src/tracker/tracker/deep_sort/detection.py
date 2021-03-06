# vim: expandtab:ts=4:sw=4
import numpy as np


class Detection(object):
    """
    This class represents a bounding box detection in a single image.

    Parameters
    ----------
    tlwh : array_like
        Bounding box in format `(x, y, w, h)`.
    confidence : float
        Detector confidence score.
    cls : str
        Class name.
    feature : array_like
        A feature vector that describes the object contained in this image.

    Attributes
    ----------
    tlwh : ndarray
        Bounding box in format `(top left x, top left y, width, height)`.
    confidence : ndarray
        Detector confidence score.
    feature : ndarray | NoneType
        A feature vector that describes the object contained in this image.

    """

    def __init__(self, tlwh, confidence, cls, feature):
        self.tlwh = np.asarray(tlwh, dtype=np.float)
        self.confidence = float(confidence)
        self.cls = cls
        self.feature = np.asarray(feature, dtype=np.float32)

    def to_tlbr(self):
        """Convert bounding box to format `(min x, min y, max x, max y)`, i.e.,
        `(top left, bottom right)`.
        """
        ret = self.tlwh.copy()
        ret[2:] += ret[:2]
        return ret

    def to_xyah(self):
        """Convert bounding box to format `(center x, center y, aspect ratio,
        height)`, where the aspect ratio is `width / height`.
        """
        ret = self.tlwh.copy()
        ret[:2] += ret[2:] / 2
        ret[2] /= ret[3]
        return ret
        
class Detection3D(object):
    """
    This class represents a 3D bounding box detection in a single space.

    Parameters
    ----------
    tlwh : array_like
        Bounding box in format `(x, y, z, w, h, l)`.
    angle : float
        3D bounding rotation angle in xy plane.
    confidence : float
        Detector confidence score.
    cls : str
        Class name.
    feature : array_like
        A feature vector that describes the object contained in this image.

    Attributes
    ----------
    tlwh : ndarray
        Bounding box in format `(top left x, top left y, top left z, width, height, lenght)`.
    confidence : ndarray
        Detector confidence score.
    feature : ndarray | NoneType
        A feature vector that describes the object contained in this space.

    """

    def __init__(self, tlwh, confidence, cls, angle, feature):
        self.tlwh = np.asarray(tlwh, dtype=np.float)
        self.confidence = float(confidence)
        self.cls = cls
        self.angle = angle
        self.feature = np.asarray(feature, dtype=np.float32) if feature is not None else []

    def to_tlbr(self):
        """Convert bounding box to format `(min x, min y, min z, max x, max y, max z)`, i.e.,
        `(top left, bottom right)`.
        """
        ret = self.tlwh.copy()
        ret[3:] += ret[:3]
        return ret

    def to_xyah(self): # to_xyzwhl
        """Convert bounding box to format `(center x, center y, center z, width,
        height, lenght)`.
        """
        ret = self.tlwh.copy()
        ret[:3] += ret[3:] / 2
        return ret
