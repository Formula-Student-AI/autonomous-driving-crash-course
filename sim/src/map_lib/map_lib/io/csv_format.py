from enum import Enum
from map_lib._io import EUFS_HEADER


class CSVFormat(list, Enum):
    EUFS = EUFS_HEADER
