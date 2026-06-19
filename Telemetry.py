import pandas as pd
import matplotlib.pyplot as plt
import math

class Telemetry:
    TPS_CLOSED = 25
    TPS_OPENED = 231
    ECT_M = 1.961
    ECT_B = -65.26
    BATT_M = 0.1047
    BATT_B = -0.369

    IDX = {
        'rpm_hi': 0, 
        'rpm_lo': 1,
        'tps': 2, 
        'tps_v': 3,
        'ect_comp': 4, 
        'ect': 5,
        'unk6': 6, 
        'unk7': 7,
        'iat_a': 8, 
        'iat': 9,
        'pad1': 10, 
        'pad2': 11,
        'batt': 12,
        'zero': 13,
        'unk14': 14,
        'unk15': 15,
        'map': 16,
        'unk17': 17,
        'unk18': 18,
        'counter':19
        }
    
    def __init__(self, csv_path):
        self.df = pd.read_csv(csv_path)
        self._clean_bytes()
        self._derive()

    def _clean_bytes(self):
        for i in range(20):
            c = f"b{1}"
            if c in self.df.columns:
                self.df[c] = (pd.to_numeric(self.df[c], errors="coerce").fillna(0).astype("int64"))

    def _derive(self):
        d = self.df
        b = lambda name: d[f"b{self.IDX[name]}"]
        
        d["rpm"] = b("rpm_hi") * 256 + b("rpm_lo")
        d["tps_pct"] = (((b("tps") - self.TPS_CLOSED) / 
                         (self.TPS_OPENED - self.TPS_CLOSED) * 100).clip(0, 100).round(1))
        d["ect_f"] = (self.ECT_M * b("ect") + self.ECT_B).round(1)
        d["ect_c"] = ((d["ect_f"] - 32) * 5/9).round(1)
        d["battery_v"] = (self.BATT_M * b("batt") + self.BATT_B).round(2)

    @classmethod
    
