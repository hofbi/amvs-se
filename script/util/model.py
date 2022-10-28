"""
Model classes for the ffmpeg experiments
"""
# pylint: disable=C0103
# Variable names should be similar as in the paper

from abc import ABCMeta, abstractmethod
from collections import namedtuple
from dataclasses import dataclass, field
from typing import Callable, List

import cv2
from sklearn.cluster import MiniBatchKMeans


@dataclass
class EncodingParameterSet:
    """Summarize all encoding parameter into one dataclass"""

    name: str
    qp: int
    rate: int
    gop_len: int
    k_size: int
    sigma: float
    resolution: str


@dataclass
class EncodingStats:
    """Summarize all results of the encoding process into one dataclass"""

    bitrate: int
    duration: float
    size: int
    sa: float
    ta: float


@dataclass
class EncodingParameters:
    """Summarize parameters used for encoding in all permutations"""

    resolution: List[str] = field(
        default_factory=lambda: [
            f"{int(352 * scale / 16)}x{int(288 * scale / 16)}" for scale in range(8, 17)
        ]
    )
    qp: List[int] = field(default_factory=lambda: list(range(24, 46)))
    rate: List[int] = field(default_factory=lambda: [30, 15, 10, 5, 3])
    gop_l: List[int] = field(
        default_factory=lambda: [
            1,
            2,
            3,
            4,
            5,
            6,
            8,
            10,
            12,
            15,
            20,
            24,
            30,
            40,
            60,
            120,
        ]
    )
    k_size: List[int] = field(default_factory=lambda: list(range(1, 11, 2)))
    sigma: List[float] = field(default_factory=lambda: [0.5 * x for x in range(1, 7)])

    @staticmethod
    def create_bd_parameters(k_size: List[int], sigma=None):
        if sigma is None:
            sigma = [0.5]
        return EncodingParameters(
            resolution=["352x288"], rate=[30], gop_l=[1], k_size=k_size, sigma=sigma
        )

    def to_list(self, video_config) -> List:
        return [
            [
                video_config["name"],
            ],
            self.qp,
            self.rate,
            self.gop_l,
            self.k_size,
            self.sigma,
            self.resolution,
        ]

    def to_filter_list(self, video_config) -> List:
        return [
            [
                video_config["name"],
            ],
            [None],
            [None],
            [None],
            self.k_size,
            self.sigma,
            [None],
        ]


class Preprocessor:
    """Define the interface for the image preprocessor"""

    __metaclass__ = ABCMeta

    @abstractmethod
    def preprocess(self, frame, params: EncodingParameterSet):
        raise NotImplementedError("Don't call me, I'm abstract.")

    @property
    @abstractmethod
    def name(self):
        raise NotImplementedError("Don't call me, I'm abstract.")


class NonePreprocessor(Preprocessor):
    """No preprocessing to generate the baseline"""

    name = "none"

    def preprocess(self, frame, params: EncodingParameterSet):
        return frame


class GaussianPreprocessor(Preprocessor):
    """Gaussian filter based preprocessing"""

    name = "gauss"

    def preprocess(self, frame, params: EncodingParameterSet):
        return cv2.GaussianBlur(frame, (params.k_size, params.k_size), params.sigma)


class ColorQuantizationAdapter(Preprocessor):
    """
    Color quantization preprocessing using k-means clustering
    Maps the k_size to the number of clusters
    """

    name = "color"

    def preprocess(self, frame, params: EncodingParameterSet):
        height, width = frame.shape[:2]
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        image = image.reshape((image.shape[0] * image.shape[1], 3))
        clt = MiniBatchKMeans(n_clusters=params.k_size)
        labels = clt.fit_predict(image)
        quant = clt.cluster_centers_.astype("uint8")[labels]
        quant = quant.reshape((height, width, 3))
        return cv2.cvtColor(quant, cv2.COLOR_LAB2BGR)


class JpegAdapter(Preprocessor):
    """
    JPEG preprocessing
    Maps the k_size to the jpeg quality level
    """

    name = "jpeg"

    def preprocess(self, frame, params: EncodingParameterSet):
        cv2.imwrite("test.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, params.k_size])
        return cv2.imread("test.jpg")


class MedianPreprocessor(Preprocessor):
    """Median filter based preprocessing"""

    name = "median"

    def preprocess(self, frame, params: EncodingParameterSet):
        return cv2.medianBlur(frame, params.k_size)


@dataclass
class Settings:
    """Settings used for the encoding step"""

    width: int = 352
    height: int = 288
    extension: str = "_cif"
    parameter: EncodingParameters = EncodingParameters()
    ffmpeg_config: Callable[
        [EncodingParameterSet], str
    ] = lambda params: f"libx264 -g {params.gop_len} -bf 0 -qp {params.qp}"
    preprocessor: Preprocessor = GaussianPreprocessor()


CombinedStats = namedtuple(
    "CombinedStats",
    list(EncodingParameterSet.__annotations__.keys())
    + list(EncodingStats.__annotations__.keys()),
)
