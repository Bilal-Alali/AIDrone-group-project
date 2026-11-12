"""2D-Belegungsraster (Log-Odds) aus Einzelstrahl-Abstandsmessungen.
Koordinaten: ENU, Meter. Zeile = Nord (y), Spalte = Ost (x). Ursprung des Rasters = Ecke (-size/2, -size/2)."""
import math
import numpy as np


class LogOddsGrid:
    def __init__(self, size_m=20.0, res=0.05, l_free=-0.4, l_occ=0.9, l_min=-5.0, l_max=5.0):
        self.res = res
        self.n = int(round(size_m / res))
        self.half = self.n * res / 2.0
        self.l_free, self.l_occ, self.l_min, self.l_max = l_free, l_occ, l_min, l_max
        self.l = np.zeros((self.n, self.n), np.float32)
        self.seen = np.zeros((self.n, self.n), bool)

    def _idx(self, e, n):
        col = np.floor((np.asarray(e) + self.half) / self.res).astype(np.int64)
        row = np.floor((np.asarray(n) + self.half) / self.res).astype(np.int64)
        ok = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return row, col, ok

    def update_ray(self, e0, n0, de, dn, dist, hit):
        """Strahl von (e0, n0) in Richtung (de, dn) (Einheitsvektor) mit Laenge dist.
        Zellen davor sind frei, bei hit=True ist die Endzelle belegt."""
        if dist <= 0:
            return
        r = np.arange(0.0, dist, self.res / 2.0)
        row, col, ok = self._idx(e0 + r * de, n0 + r * dn)
        flat = np.unique(row[ok] * self.n + col[ok])
        if hit:
            er, ec, eok = self._idx(e0 + dist * de, n0 + dist * dn)
            end = int(er) * self.n + int(ec) if bool(eok) else None
            if end is not None:
                flat = flat[flat != end]
        lf = self.l.reshape(-1)
        sf = self.seen.reshape(-1)
        lf[flat] = np.maximum(lf[flat] + self.l_free, self.l_min)
        sf[flat] = True
        if hit and end is not None:
            lf[end] = min(lf[end] + self.l_occ, self.l_max)
            sf[end] = True

    def occupancy(self):
        """int8-Raster fuer nav_msgs/OccupancyGrid: -1 unbekannt, 0 frei ... 100 belegt."""
        p = 1.0 / (1.0 + np.exp(-1.5 * self.l))
        occ = (100.0 * p).astype(np.int8)
        occ[~self.seen] = -1
        return occ
