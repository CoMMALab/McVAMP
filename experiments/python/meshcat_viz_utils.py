"""Generic meshcat/pinocchio visualization helpers (MeshcatViz class) for any URDF robot.
Used by maze_meshcat_example.py. Needs pinocchio, meshcat, matplotlib."""
import time
import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer

import meshcat.geometry as mg
import matplotlib.pyplot as plt

from meshcat.transformations import translation_matrix


import tempfile

def replace_package_path(urdf_path):
    '''
    Take in a urdf path, search for instances of package://franka_description,
    and then replace with the working directory path.
    
    Args:
        urdf_path (str): The path to the URDF file.
    
    Returns:
        str: The modified URDF path with package paths replaced.
    '''
    import os
    import re
    
    # Get all parents in the path
    parents = os.path.dirname(urdf_path).split(os.sep)
    replace_with = os.path.join(os.getcwd(), *parents) + '/meshes/'
        
    # Read the URDF file
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # Replace package://franka_description with the current working directory
    modified_content = re.sub(r'package://franka_description/meshes/', replace_with, urdf_content)
    # modified_content = re.sub(r'package://meshes/', replace_with, urdf_content)

    # some urdfs have "package://franka_description/meshes/..." and some have meshes/ without the package prefix.
    # Uncomment the following if your case is the latter, and comment out the above replacement.
    # modified_content = re.sub(r'meshes/', replace_with, urdf_content)
    
    # Write to a temp file
    tmp = tempfile.NamedTemporaryFile(delete=False)
    with open(tmp.name, 'w') as f:
        f.write(modified_content)
        
    return tmp.name
    



# roll pitch yaw
axis_directions = np.pi/2 * np.array([[1, 0, 0], [0, 0, 0], [0, 0, 1]])

axis_colors = [[80, 230, 100],   # Green
               [255, 140, 0],    # Orange
               [190, 80, 250]]   # Purple 

def color_to_hex(color):
    '''
    convert RGB color in [0, 255] to int hex
    '''
    return (color[0] << 16) + (color[1] << 8) + color[2]


class MeshcatViz:
    def __init__(self):
        self.viz = None
        self.model = None
        self.axs = None
        self.waypoints_set = 'a'

    def init_viz(self, robot_urdf_path=None, robot_mesh_path=None):
        # Load robot model
        self.model, coll_model, vis_model = pinocchio.buildModelsFromUrdf(
            replace_package_path(robot_urdf_path), robot_mesh_path, )

        # Set up visualizer
        self.viz = MeshcatVisualizer(self.model, coll_model, vis_model)
        self.viz.initViewer(zmq_url="tcp://127.0.0.1:6000")
        self.viz.loadViewerModel()
        
        fig, self.axs = plt.subplots(2, 3, sharex=True, sharey='row', figsize=(12, 8))
        plt.tight_layout(pad=1.0)
        plt.ion()
        
        return self.viz

    def clear_all_waypoints(self):
        '''
        Clear all waypoints from both sets.
        '''
        self.viz.viewer["waypoints_a"].delete()
        self.viz.viewer["waypoints_b"].delete()
        self.viz.viewer["waypoints"].delete()

    def render_eefs(self, eeposes):
        """
        Renders the trajectory of a specific frame as a series of coordinate axes.

        Parameters
        ----------
        eeposes
        """
        visualizer = self.viz
        # model = self.viz.model
        self.clear_all_waypoints()
        
        # Draw waypoints as spheres
        for i, eepose in enumerate(eeposes):
            visualizer.viewer[f"waypoints_{self.waypoints_set}/waypoint_{i}"].set_object(
                mg.Sphere(0.005),
                mg.MeshLambertMaterial(color=color_to_hex([255, 140, 0]))
            )
            eepose[2, 3] -= 0.16
            visualizer.viewer[f"waypoints_{self.waypoints_set}/waypoint_{i}"].set_transform(eepose)
        
        # Toggle the other set of waypoints and delete it
        # (We do this so there isn't a flash when updating)
        self.waypoints_set = 'b' if self.waypoints_set == 'a' else 'a'
        visualizer.viewer[f"waypoints_{self.waypoints_set}"].delete()

    def add_cuboids(self, cuboids, colors=(12, 89, 178)):
        '''
        cuboids: list of dicts with [x, y, z, yaw, pitch, roll, dx, dy, dz]
        colors: list of RGB tuples in [0, 255] or a single RGB tuple
        '''

        visualizer = self.viz
        for i, cuboid in enumerate(cuboids):
            x, y, z = cuboid[0], cuboid[1], cuboid[2]
            yaw, pitch, roll = cuboid[3], cuboid[4], cuboid[5]
            dx, dy, dz = cuboid[6], cuboid[7], cuboid[8]
            
            # Create transformation matrix
            T = translation_matrix([x, y, z])
            # Rz = pinocchio.rpy.rpyToMatrix([0, 0, yaw])
            # Ry = pinocchio.rpy.rpyToMatrix([0, pitch, 0])
            # Rx = pinocchio.rpy.rpyToMatrix([roll, 0, 0])
            # R = Rz @ Ry @ Rx
            # T[:3, :3] = R
            
            visualizer.viewer[f"cuboids/cuboid_{i}"].set_object(
                mg.Box([dx, dy, dz]),
                mg.MeshLambertMaterial(color=color_to_hex(colors[i] if isinstance(colors, list) else colors))
            )
            # visualizer.viewer[f"cuboids/cuboid_{i}/edges"].set_object(
            #     mg.Box([dx, dy, dz]),
            #     mg.MeshBasicMaterial(color=0x000000, wireframe=True)
            # )

            visualizer.viewer[f"cuboids/cuboid_{i}"].set_transform(T)

    def animate(self, positions, all_times, rate=1.0, loop=False):
        '''
        Generate a nice interpolated trajectory animation.
        '''
        # Get interpolated trajectory
        # all_times, positions = interpolate_trajectory(qs, vs, dts, time_step=0.01)
        
        # Assert all_times is strictly increasing
        assert np.all(np.diff(all_times) > 0), "all_times must be strictly increasing"
        
        padded_positions = np.zeros((positions.shape[0], self.model.nq))
        padded_positions[:, :positions.shape[1]] = positions
        # Animate
        start_time = time.perf_counter()
        i = 0
        while True:
            # Move to the right time
            elapsed = (time.perf_counter() - start_time) * rate
            if elapsed > all_times[-1]:
                start_time = time.perf_counter()
                if loop: continue
                else: break
            
            # Bisect find i
            i = np.searchsorted(all_times, elapsed, side="right") - 1
            i = np.clip(i, 0, positions.shape[0] - 1)
            
            self.viz.display(padded_positions[i]) # assume 7
            
        self.viz.viewer["waypoints"].delete()

if __name__ == '__main__':
    traj = np.loadtxt("trajectory.txt", delimiter = ",")
    meshcat_viz = MeshcatViz()
    meshcat_viz.init_viz()
    meshcat_viz.clear_all_waypoints()
    print(traj.shape)
    meshcat_viz.animate(traj, np.arange(0, len(traj), dtype=np.float64) / 100, loop=True)