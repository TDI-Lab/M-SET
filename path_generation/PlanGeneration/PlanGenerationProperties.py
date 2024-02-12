from dataclasses import dataclass


@dataclass
class PlanGenerationProperties:
    dataset_name: str
    plans_num: int
    agents_num: int
    total_hover_time: float
    max_visited_cells_num: int
    stations_num: int
    height: float
    map_length: int
    battery_capacity: float
