from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GridConfig:
    cols: int = 4
    rows: int = 4

    def section_of(self, x: float, y: float, width: int, height: int) -> int:
        """Return 1..(cols*rows) section index (row-major).

        Sections:
          row1:  1  2  3  4
          row2:  5  6  7  8
          row3:  9 10 11 12
          row4: 13 14 15 16
        """
        if width <= 0 or height <= 0:
            return 0
        x = max(0.0, min(float(x), float(width - 1)))
        y = max(0.0, min(float(y), float(height - 1)))

        col = int(x / (width / self.cols))
        row = int(y / (height / self.rows))
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return row * self.cols + col + 1
