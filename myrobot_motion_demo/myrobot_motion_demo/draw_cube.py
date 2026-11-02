#!/usr/bin/env python3
"""ROS 2 node for demonstrating MoveIt 2 planning by drawing a cube."""

import os
import time
from typing import List, Dict, Final, Optional
import numpy as np

import rclpy
from rclpy.logging import get_logger
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Pose
from moveit.planning import MoveItPy, PlanRequestParameters, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder

class MoveItDemo:
    """A demonstration class for MoveIt 2 motion planning in ROS 2.

    This class initializes MoveItPy, plans a path to an initial joint
    configuration using the OMPL planning pipeline, and then executes a
    Cartesian path tracing a cube using the Pilz Industrial Motion Planner.
    """
    
    # Constants
    PLANNING_GROUP: Final[str] = "arm"
    END_EFFECTOR_LINK: Final[str] = "Link5"
    BASE_FRAME: Final[str] = "base_link"
    
    # Planning pipeline constants
    OMPL_PIPELINE: Final[str] = "ompl"
    OMPL_PLANNER: Final[str] = "RRTConnectkConfigDefault"
    PILZ_PIPELINE: Final[str] = "pilz_industrial_motion_planner"
    PILZ_LIN_PLANNER: Final[str] = "LIN"
    
    # Speed/acceleration scaling constants
    VELOCITY_SCALE: Final[float] = 0.1
    ACCELERATION_SCALE: Final[float] = 0.1
    
    # Joint values target (initial pose)
    INITIAL_JOINT_POSITIONS: Final[Dict[str, float]] = {
        "Joint1": 0.0,
        "Joint2": -0.0662000911461428,
        "Joint3": 1.0649385261805382,
        "Joint4": 0.2596219269972221,
        "Joint5": -1.2583603953352513
    }
    
    # Cube parameters
    CUBE_SIDE_LENGTH_METERS: Final[float] = 0.04
    
    # Sleep duration between steps
    STEP_SLEEP_SECONDS: Final[float] = 1.0

    # Instance variables
    logger: RcutilsLogger
    clock: Clock
    moveit: MoveItPy
    planning_component: PlanningComponent

    def __init__(self) -> None:
        """Initializes the MoveItDemo instance.

        Loads the MoveIt parameters, initializes the MoveItPy interface,
        and retrieves the planning component for the robot arm.
        """
        self.logger = get_logger("draw_cube")
        self.clock = Clock()
        self.logger.info("Building MoveIt configurations...")
        
        # Locate moveit_py.yaml configuration file
        config_file_path: str = os.path.join(
            get_package_share_directory("myrobot_motion_demo"),
            "config",
            "moveit_py.yaml"
        )
        
        # Build MoveIt configuration dictionary including moveit_cpp options
        moveit_config: Dict = (
            MoveItConfigsBuilder("myrobot_description", package_name="myrobot_moveit_config")
            .moveit_cpp(file_path=config_file_path)
            .to_moveit_configs()
            .to_dict()
        )
        
        self.logger.info("Initializing MoveItPy instance with configuration dictionary...")
        # Instantiate MoveItPy directly with the configuration dictionary
        self.moveit = MoveItPy(node_name="draw_cube", config_dict=moveit_config)
        
        # Get the planning component for the arm
        self.planning_component = self.moveit.get_planning_component(self.PLANNING_GROUP)
        self.logger.info("MoveItPy initialization completed.")

    def move_to_initial_joints(self) -> bool:
        """Commands the robot to the initial joint configuration using OMPL.

        Returns:
            bool: True if planning and execution succeeded, False otherwise.
        """
        self.logger.info("Planning to initial joint positions using OMPL...")
        
        # Configure OMPL plan parameters using PlanRequestParameters
        ompl_params: PlanRequestParameters = PlanRequestParameters(
            self.moveit, self.OMPL_PIPELINE
        )
        ompl_params.planning_pipeline = self.OMPL_PIPELINE
        ompl_params.planner_id = self.OMPL_PLANNER
        ompl_params.max_velocity_scaling_factor = self.VELOCITY_SCALE
        ompl_params.max_acceleration_scaling_factor = self.ACCELERATION_SCALE
        
        # Set target state using a RobotState object
        robot_model = self.moveit.get_robot_model()
        goal_state: RobotState = RobotState(robot_model)
        
        # Map initial positions to list ordered by active joint names
        joint_names: List[str] = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5"]
        joint_positions: List[float] = [
            self.INITIAL_JOINT_POSITIONS[name] for name in joint_names
        ]
        
        goal_state.set_joint_group_positions(
            self.PLANNING_GROUP, np.array(joint_positions, dtype=np.float64)
        )
        
        # Set start and goal state on the planning component
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(robot_state=goal_state)
        
        # Plan and execute
        plan_result = self.planning_component.plan(single_plan_parameters=ompl_params)
        if not plan_result:
            self.logger.error("Failed to plan to initial joint positions!")
            return False
            
        success: bool = self.moveit.execute(plan_result.trajectory, controllers=[])
        if success:
            self.logger.info("Successfully reached initial joint positions.")
        else:
            self.logger.error("Execution to initial joint positions failed!")
        return success

    def draw_cube(self) -> bool:
        """Draws a cube in Cartesian space using the Pilz LIN planner.

        Returns:
            bool: True if the cube was successfully traced, False otherwise.
        """
        self.logger.info("Drawing cube in Cartesian space using Pilz LIN...")
        
        # Access the current state using the PlanningSceneMonitor
        psm = self.moveit.get_planning_scene_monitor()
        with psm.read_only() as scene:
            current_state: RobotState = scene.current_state
            start_pose: Pose = current_state.get_pose(self.END_EFFECTOR_LINK)
        
        start_x: float = start_pose.position.x
        start_y: float = start_pose.position.y
        start_z: float = start_pose.position.z
        
        self.logger.info(
            f"Start pose Cartesian coords: x={start_x:.4f}, y={start_y:.4f}, z={start_z:.4f}"
        )
        
        # Define 8 vertices of the cube relative to start pose
        cube_side: float = self.CUBE_SIDE_LENGTH_METERS
        vertices: List[List[float]] = [
            [start_x, start_y, start_z],                                     # V0
            [start_x + cube_side, start_y, start_z],                         # V1
            [start_x + cube_side, start_y + cube_side, start_z],             # V2
            [start_x, start_y + cube_side, start_z],                         # V3
            [start_x, start_y, start_z + cube_side],                         # V4
            [start_x + cube_side, start_y, start_z + cube_side],             # V5
            [start_x + cube_side, start_y + cube_side, start_z + cube_side], # V6
            [start_x, start_y + cube_side, start_z + cube_side]              # V7
        ]
        
        # Index path that traces all 12 edges of the cube
        vertex_path_indices: List[int] = [0, 1, 2, 3, 0, 4, 5, 6, 7, 4, 5, 1, 2, 6, 7, 3]
        
        # Configure Pilz request parameters
        pilz_params: PlanRequestParameters = PlanRequestParameters(
            self.moveit, self.PILZ_PIPELINE
        )
        pilz_params.planning_pipeline = self.PILZ_PIPELINE
        pilz_params.planner_id = self.PILZ_LIN_PLANNER
        pilz_params.max_velocity_scaling_factor = self.VELOCITY_SCALE
        pilz_params.max_acceleration_scaling_factor = self.ACCELERATION_SCALE
        
        # Calculate the starting radial yaw angle
        import math
        start_yaw: float = math.atan2(start_y, start_x)
        
        # Execute movement to each vertex in the path
        step_number: int = 1
        for vertex_index in vertex_path_indices:
            target_coords: List[float] = vertices[vertex_index]
            self.logger.info(
                f"Step {step_number}/{len(vertex_path_indices)}: Planning LIN to vertex V{vertex_index} "
                f"at coordinates: x={target_coords[0]:.4f}, y={target_coords[1]:.4f}, z={target_coords[2]:.4f}"
            )
            
            # Construct target pose message keeping start orientation
            pose_goal: PoseStamped = PoseStamped()
            pose_goal.header.frame_id = self.BASE_FRAME
            pose_goal.header.stamp = self.clock.now().to_msg()
            pose_goal.pose.position.x = target_coords[0]
            pose_goal.pose.position.y = target_coords[1]
            pose_goal.pose.position.z = target_coords[2]
            
            # Compute yaw rotation to align end-effector with the radial vector to target coords
            target_yaw: float = math.atan2(target_coords[1], target_coords[0])
            delta_yaw: float = target_yaw - start_yaw
            
            # Rotate starting orientation quaternion by delta_yaw around Z-axis
            half_delta: float = delta_yaw / 2.0
            sin_half: float = math.sin(half_delta)
            cos_half: float = math.cos(half_delta)
            
            q1_x: float = 0.0
            q1_y: float = 0.0
            q1_z: float = sin_half
            q1_w: float = cos_half
            
            q2_x: float = start_pose.orientation.x
            q2_y: float = start_pose.orientation.y
            q2_z: float = start_pose.orientation.z
            q2_w: float = start_pose.orientation.w
            
            # Apply Hamilton product to compute the rotated orientation
            pose_goal.pose.orientation.x = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y
            pose_goal.pose.orientation.y = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x
            pose_goal.pose.orientation.z = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w
            pose_goal.pose.orientation.w = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z
            
            # Set target goal
            self.planning_component.set_start_state_to_current_state()
            self.planning_component.set_goal_state(
                pose_stamped_msg=pose_goal,
                pose_link=self.END_EFFECTOR_LINK
            )
            
            # Plan trajectory
            plan_result = self.planning_component.plan(single_plan_parameters=pilz_params)
            if not plan_result:
                self.logger.error(f"Failed to plan Cartesian LIN path to V{vertex_index}!")
                return False
                
            # Execute trajectory
            execute_success: bool = self.moveit.execute(plan_result.trajectory, controllers=[])
            if not execute_success:
                self.logger.error(f"Failed to execute path to V{vertex_index}!")
                return False
                
            self.logger.info(f"Reached V{vertex_index} successfully.")
            step_number += 1
            time.sleep(self.STEP_SLEEP_SECONDS)
            
        self.logger.info("Successfully drew the cube in space!")
        return True

def main(args: Optional[List[str]] = None) -> None:
    """Main execution function for the MoveIt demonstration node.

    Args:
        args: Command line arguments passed to rclcpp.
    """
    rclpy.init(args=args)
    demo: MoveItDemo = MoveItDemo()
    
    # Execute the tasks
    try:
        if demo.move_to_initial_joints():
            time.sleep(demo.STEP_SLEEP_SECONDS)
            demo.draw_cube()
    except Exception as error:
        get_logger("draw_cube").error(f"An unexpected error occurred during execution: {error}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
