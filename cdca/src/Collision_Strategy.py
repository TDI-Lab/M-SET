from abc import ABC, abstractmethod 
  
class Collision_Strategy(ABC): 
    # Abstract base class for different collision avoidance strategies.
    
    @abstractmethod
    def detect_potential_collisions(self, drones): 
        # The interface collision avoidance algorithms need to implement.
        pass
