"""Parsing buffer for Viomi map data."""

import logging
from struct import unpack_from

_LOGGER = logging.getLogger(__name__)


class ParsingBuffer:
    """Parsing buffer for Viomi map data."""

    def __init__(self, name: str, data: bytes, start_offs: int, length: int):
        self._name = name
        self.data = data
        self.offs = start_offs
        self.length = length
        self._image_beginning: int = 0

    def set_name(self, name: str) -> None:
        self._name = name
        _LOGGER.debug("SECTION %s: offset 0x%x", self._name, self.offs)

    def mark_as_image_beginning(self) -> None:
        self._image_beginning = self.offs

    def get_at_image(self, offset: int) -> int:
        return self.data[self._image_beginning + offset - 1]

    def skip(self, field: str, n: int) -> None:
        if self.length < n:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        self.offs += n
        self.length -= n

    def get_uint8(self, field: str) -> int:
        if self.length < 1:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        self.offs += 1
        self.length -= 1
        return self.data[self.offs - 1]

    def get_uint16(self, field: str) -> int:
        if self.length < 2:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        value = self._unpack_int("<H")
        self.offs += 2
        self.length -= 2
        return value

        
    def get_uint16_remove_parity(self, field: str) -> int:
        if self.length < 2:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self._offs:#x}: buffer underrun")
        lo = self.data[self.offs] 
        hi = self.data[self.offs + 1]
        self.offs += 2
        self.length -= 2
        return ((hi^1) << 7) ^ lo
        
    def get_uint32(self, field: str) -> int:
        value = self.peek_uint32(field)
        self.offs += 4
        self.length -= 4
        return value

    def get_float32(self, field: str) -> float:
        if self.length < 4:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        self.offs += 4
        self.length -= 4
        return float(unpack_from("<f", self.data, self.offs - 4)[0])

    def get_string_len8(self, field: str) -> str:
        n = self.get_uint8(field + ".len")
        if self.length < n:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        self.offs += n
        self.length -= n
        return self.data[self.offs - n : self.offs].decode("UTF-8")

    def peek_uint32(self, field: str) -> int:
        if self.length < 4:
            raise ValueError(f"error parsing {self._name}.{field} at offset {self.offs:#x}: buffer underrun")
        return self._unpack_int("<L")

    def check_empty(self) -> None:
        if self.length == 0:
            _LOGGER.debug("all of the data has been processed")
        else:
            _LOGGER.warning("%d bytes remained in the buffer", self.length)

    def _unpack_int(self, fmt: str) -> int:
        return int(unpack_from(fmt, self.data, self.offs)[0])

    def _unpack_float(self, fmt: str) -> float:
        return float(unpack_from(fmt, self.data, self.offs)[0])
