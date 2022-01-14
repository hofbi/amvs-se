"""
Model classes for the ffmpeg experiments
"""
# pylint: disable=C0103
# Variable names should be similar as in the paper

from collections import namedtuple
from dataclasses import dataclass, field
from typing import List, Callable


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

    def to_list(self, video_config):
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

    def to_filter_list(self, video_config):
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


CombinedStats = namedtuple(
    "CombinedStats",
    list(EncodingParameterSet.__annotations__.keys())
    + list(EncodingStats.__annotations__.keys()),
)
