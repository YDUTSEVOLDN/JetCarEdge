from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class EncodedImage:
    width: int
    height: int
    data: str
    encoding: str = "jpeg"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "encoding": self.encoding,
            "width": self.width,
            "height": self.height,
            "data": self.data,
        }


@dataclass(frozen=True)
class EdgeFrame:
    car_id: str
    timestamp: float
    image: EncodedImage
    sensors: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "edge_frame",
            "car_id": self.car_id,
            "timestamp": self.timestamp,
            "image": self.image.to_dict(),
            "sensors": self.sensors,
        }


@dataclass(frozen=True)
class VideoFrameUpload:
    car_id: str
    image: EncodedImage

    def to_dict(self) -> Dict[str, Any]:
        return {
            "car_id": self.car_id,
            "image": self.image.to_dict(),
        }
