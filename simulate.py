# Written with assistance from Github Co-Pilot

import time
import dm_control.mujoco
import mujoco.viewer

from creature import CreatureGenerator

def main():
    cg = CreatureGenerator()
    creature = cg.generate_creature()

    with open('creature.xml', 'w') as f:
        f.write(creature.model.to_xml_string())
    
    m = dm_control.mujoco.MjModel.from_xml_path("creature.xml")
    d = dm_control.mujoco.MjData(m)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Set camera parameters
        # These parameters can be adjusted to change the camera angle and perspective
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -20  # Elevation angle (in degrees)
        viewer.cam.distance = 3.0  # Distance from the camera to the target
        viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
        viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
        viewer.cam.lookat[2] = 0.75  # Z-coordinate of the target position

        for i in range(1000):
            dm_control.mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(1/100)

        viewer.close()


if __name__ == "__main__":
    main()