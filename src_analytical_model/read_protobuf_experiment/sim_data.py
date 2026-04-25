import os
import sys
from google.protobuf.internal.decoder import _DecodeVarint32

# Import compiled protobuf messages
import read_protobuf_experiment.time_step_pb2 as pb2
#import time_step_pb2 as pb2

class SimData:
    """
    Wrapper class for reading simulation logs stored in Protobuf format.
    """

    def __init__(self, log_data_path):
        """
        Initialize a SimData instance.

        Args:
            log_data_path (str): Path to the binary protobuf log file (.pb).
        """
        self.data = {}

        # Parse protobuf log data
        self.read_log_data(log_data_path)

    # -------------------------------------------------------------------------
    # LOG PARSING
    # -------------------------------------------------------------------------
    def read_log_data(self, log_data_path):
        """Parses the binary protobuf simulation log file."""
        with open(log_data_path, "rb") as f:
            buf = f.read()
            n = 0

            # Read each TimeStep message (prefixed with varint length)
            while n < len(buf):
                msg_len, new_pos = _DecodeVarint32(buf, n)
                n = new_pos
                msg_buf = buf[n : n + msg_len]
                n += msg_len

                timestep = pb2.TimeStep()
                timestep.ParseFromString(msg_buf)

                # Store TimeStep object
                self.data[timestep.time] = {"log": timestep}

        if not self.data:
            raise ValueError("No TimeStep entries found in log file.")

        self.totalTime = len(self.data)

        # Handle real-robot logs where last entry may be missing
        if self.totalTime not in self.data:
            self.totalTime -= 1

        last_log = self.data[self.totalTime*10]["log"]
        self.totalPoints = getattr(last_log, "points", None)

        # Number of robots in the first recorded timestep
        first_log = self.data[list(self.data.keys())[0]]["log"]
        self.numWorkers = len(first_log.robots)

        # Determine number of tasks based on task IDs
        self.numTasks = max(
            (int(task.name[5:]) for task in last_log.tasks if task.name.startswith("task_")),
            default=0,
        )

    # -------------------------------------------------------------------------
    # ACCESSORS
    # -------------------------------------------------------------------------
    def __getitem__(self, key):
        """Allow dictionary-style access (e.g., sim_data[100])"""
        return self.data[key]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return f"<SimData: {len(self.data)} timesteps, {self.numWorkers} robots, {self.numTasks} tasks>"
