import os
from datetime import datetime
from typing import Optional


def get_most_recent_experiment_directory_name(base_directory):
    """
        Finds the most recent directory in the base directory based on the datetime in its name.
        Directory names should follow the format: data_%d_%m_%Y_%H_%M_%S.

        Args:
            base_directory (str): Path to the base directory to search.

        Returns:
            str: The name of the most recent directory, or None if no valid directories are found.
        """
    most_recent_datetime = None
    most_recent_directory = None

    for entry in os.listdir(base_directory):
        full_path = os.path.join(base_directory, entry)

        if os.path.isdir(full_path):

            try:
                # assumes directory names are of the format: data_%d_%m_%Y_%H_%M_%S
                datetime_part = entry.split('_', 1)[1]

                dir_datetime = datetime.strptime(datetime_part, '%d_%m_%Y_%H_%M_%S')

                if most_recent_datetime is None or dir_datetime > most_recent_datetime:
                    most_recent_datetime = dir_datetime
                    most_recent_directory = full_path

            except (IndexError, ValueError):
                continue  # skip directories that don't match the expected format

    return most_recent_directory


def get_experiment_folder_path(base_directory, folder: str = None) -> Optional[str]:
    """
    Returns an experiment folder according to the following criteria:
    * if a folder is specified, then the path to this folder is returned
    * in case no folder is specified, the path to the most recent experiment is returned
    """

    if folder is None:
        return get_most_recent_experiment_directory_name(base_directory)
    else:
        return os.path.join(base_directory, folder)
