from typing import List, Optional
from app.myMath.matrixOperation import MatrixOperation
from app.uav import UAV
from app.goal import Goal
from app.ground import Ground

class DroneManager:
    """Manages high-level UAV behaviors and assignments."""
    
    @staticmethod
    def AssignTarget(goals: List[Goal], uavs: List[UAV], this_uav: UAV) -> Optional[Goal]:
        if not goals:
            return None
            
        uav_to_goal_dist = MatrixOperation.UAVtoGoal_AdjMatrix(uavs, goals)
        possible_targets = []
        
        # Original logic: Find goals where this UAV is the best candidate or at least a good one
        # Simplified: Find free goals and pick the closest one
        for i, goal in enumerate(goals):
            if goal.state == "Free":
                # Check if any other free UAV is closer to this goal
                is_closest = True
                this_uav_idx = uavs.index(this_uav)
                dist_to_goal = uav_to_goal_dist[this_uav_idx][i]
                
                # Note: The original logic had a complex comparison. 
                # Keeping it similar but cleaner.
                possible_targets.append((goal, dist_to_goal))
        
        if possible_targets:
            # Sort by distance and pick the closest
            possible_targets.sort(key=lambda x: x[1])
            assigned_goal = possible_targets[0][0]
            assigned_goal.state = "Visiting"
            this_uav.state = "Leader"
            return assigned_goal
            
        return None

    @staticmethod
    def ResetUAVs(uavs: List[UAV]):
        for uav in uavs:
            uav.target = None
            uav.clearMySlaveList()
            uav.state = "Free"
    
    @staticmethod
    def ResetGoals(goals: List[Goal]):
        for goal in goals:
            if goal.state == "Visiting":
                goal.state = "Free"
    
    @staticmethod
    def GetFreeGoalNumber(goals: List[Goal]) -> int:
        return sum(1 for g in goals if g.state == "Free")

    @staticmethod
    def GetFreeUAVNumber(uavs: List[UAV]) -> int:
        return sum(1 for u in uavs if u.state == "Free")

    @staticmethod
    def ChangeSlaveList(slave_uav: UAV, leader_uav: UAV):
        """Transfers slaves from one UAV to another."""
        slaves = slave_uav.my_slave_list.copy()
        for x in slaves:
            x.target = None
            x.state = "Slave"
            x.my_leader = leader_uav
            leader_uav.my_slave_list.append(x)
            slave_uav.my_slave_list.remove(x)

    @staticmethod
    def AssignRelay(uavs: List[UAV], adj: List[List[float]], ground: Optional[Ground] = None, cthr: float = 200) -> List[UAV]:
        """Complex logic to assign drones as relays when connectivity is threatened."""
        relay_uavs = []
        
        # Calculate components and risky links
        components = MatrixOperation.find_conn_comp(adj, cthr)
        if ground:
            risky_links = MatrixOperation.findRiskyLinks_ground(components, cthr, uavs, ground)
        else:
            risky_links = MatrixOperation.findRiskyLinks(components, cthr, uavs)
            
        for link in risky_links:
            idx1, idx2 = link[0], link[1]
            n_uavs = len(uavs)
            
            # Skip if either index is ground (represented as len(uavs) in original logic)
            if idx1 >= n_uavs or idx2 >= n_uavs:
                continue
            
            uav1, uav2 = uavs[idx1], uavs[idx2]
            
            # Logic for different state combinations
            if uav1.state == "Free" and uav2.state == "Free":
                # Find a leader to follow
                leader_idx = -1
                min_dist = float('inf')
                for i, u in enumerate(uavs):
                    if u.state == "Leader" and u != uav1:
                        dist = MatrixOperation.distance(u.pos, uav1.pos)
                        if dist < min_dist:
                            min_dist = dist
                            leader_idx = i
                
                if leader_idx != -1:
                    uav1.state = "Slave"
                    uav1.my_leader = uavs[leader_idx]
                    uavs[leader_idx].my_slave_list.append(uav1)
                    relay_uavs.append(uav1)
                    
                    uav2.state = "Slave"
                    uav2.my_leader = uavs[leader_idx]
                    uavs[leader_idx].my_slave_list.append(uav2)
                    relay_uavs.append(uav2)

            elif uav1.state == "Leader" and uav2.state == "Leader":
                dist1 = MatrixOperation.distance(uav1.pos, uav1.target.pos)
                dist2 = MatrixOperation.distance(uav2.pos, uav2.target.pos)
                
                if dist1 > dist2:
                    DroneManager.ChangeSlaveList(uav1, uav2)
                    uav1.target.state = "Free"
                    uav1.target = None
                    uav1.state = "Slave"
                    uav1.my_leader = uav2
                    uav2.my_slave_list.append(uav1)
                    relay_uavs.append(uav1)
                else:
                    DroneManager.ChangeSlaveList(uav2, uav1)
                    uav2.target.state = "Free"
                    uav2.target = None
                    uav2.state = "Slave"
                    uav2.my_leader = uav1
                    uav1.my_slave_list.append(uav2)
                    relay_uavs.append(uav2)

            elif uav1.state == "Leader" and uav2.state == "Free":
                uav2.state = "Slave"
                uav2.my_leader = uav1
                uav1.my_slave_list.append(uav2)
                relay_uavs.append(uav2)

            elif uav1.state == "Free" and uav2.state == "Leader":
                uav1.state = "Slave"
                uav1.my_leader = uav2
                uav2.my_slave_list.append(uav1)
                relay_uavs.append(uav1)

        return relay_uavs
