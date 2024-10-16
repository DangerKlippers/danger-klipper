# Shake&Tune: 3D printer analysis tools
#
# Copyright (C) 2024 Félix Boisselier <felix@fboisselier.fr> (Frix_x on Discord)
# Licensed under the GNU General Public License v3.0 (GPL-3.0)
#
# File: shaketune_config.py
# Description: Defines the ShakeTuneConfig class for handling configuration settings
#              and file paths related to Shake&Tune operations.


from pathlib import Path

KLIPPER_FOLDER = Path.home() / "klipper"
KLIPPER_LOG_FOLDER = Path.home() / "printer_data/logs"
RESULTS_BASE_FOLDER = Path.home() / "printer_data/config/K-ShakeTune_results"
RESULTS_SUBFOLDERS = {
    "axes map": "axes_map",
    "belts comparison": "belts",
    "input shaper": "input_shaper",
    "vibrations profile": "vibrations",
    "static frequency": "static_freq",
}


class ShakeTuneConfig:
    def __init__(
        self,
        result_folder: Path = RESULTS_BASE_FOLDER,
        keep_n_results: int = 3,
        keep_csv: bool = False,
        dpi: int = 150,
        git_info: dict = {},
    ) -> None:
        self._result_folder = result_folder

        self.keep_n_results = keep_n_results
        self.keep_csv = keep_csv
        self.dpi = dpi
        self.version = f"Danger-Klipper {git_info['version']} ({git_info['branch']} on {git_info['remote']})"

        self.klipper_folder = KLIPPER_FOLDER
        self.klipper_log_folder = KLIPPER_LOG_FOLDER

    def get_results_folder(self, type: str = None) -> Path:
        if type is None:
            return self._result_folder
        else:
            return self._result_folder / RESULTS_SUBFOLDERS[type]

    def get_results_subfolders(self) -> Path:
        subfolders = [
            self._result_folder / subfolder
            for subfolder in RESULTS_SUBFOLDERS.values()
        ]
        return subfolders
