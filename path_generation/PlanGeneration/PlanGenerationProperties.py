from dataclasses import dataclass


@dataclass
class PlanGenerationProperties:
    dataset_name: str
    plans_num: int
    agents_num: int
    total_hover_time: float
    path_mode: str
    stations_num: int
    height: float
    map_length: int
    battery_capacity: float
