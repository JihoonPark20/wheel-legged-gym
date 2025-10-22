from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class B2WRoughCfg( LeggedRobotCfg ):
    class env:
        num_envs = 4096
        num_observations = 243
        num_privileged_obs = None
        num_actions = 16
        env_spacing = 3.
        send_timeouts = True
        episode_length_s = 20
    
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.54] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.0, 
            'RL_hip_joint': 0.0, 
            'FR_hip_joint': -0.0, 
            'RR_hip_joint': -0.0, 

            'FL_thigh_joint': 0.8, 
            'RL_thigh_joint': 0.8, 
            'FR_thigh_joint': 0.8, 
            'RR_thigh_joint': 0.8, 

            'FL_calf_joint': -1.5, 
            'RL_calf_joint': -1.5, 
            'FR_calf_joint': -1.5, 
            'RR_calf_joint': -1.5, 

            'FL_foot_joint': 0.0,
            'RL_foot_joint': 0.0,
            'FR_foot_joint': 0.0,
            'RR_foot_joint': 0.0,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        stiffness = {'hip_joint': 160.,'thigh_joint': 160.,'calf_joint': 160.,"foot_joint": 0.0} 
        damping = {'hip_joint': 5.0,'thigh_joint': 5.0,'calf_joint': 5.0,"foot_joint": 1.0} 
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        hip_scale_reduction = 0.5
        vel_scale = 10.0
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/b2w_description/urdf/b2w_description.urdf'
        name = "b2w"
        foot_name = "foot"
        wheel_radius = 0.225
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base", "hip"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = True
        fix_base_link = False
  
    class rewards:
        soft_dof_pos_limit = 0.9
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 1.
        base_height_target = 0.5
        tracking_sigma = 0.4
        max_contact_force = 200.
        wheeled_torque_scale = 0.0
        wheeled_joint_power_scale = 0.0
        wheeled_dof_vel_scale = 0.0
        wheeled_dof_acc_scale = 0.0
        wheeled_action_rate_scale = 0.0
        only_positive_rewards = False
        class scales:
            termination = -0.8
            tracking_lin_vel = 3.0
            tracking_ang_vel = 2.0
            lin_vel_z = -0.1
            ang_vel_xy = -0.05
            orientation = -2.0
            torques = -0.0001
            dof_vel = -1e-7
            dof_acc = -1e-7
            base_height = -0.5 
            feet_air_time =  0.0
            collision = -0.1
            stumble = -0.1 
            action_rate = -0.0002
            stand_still = -0.01
            feet_contact_forces = -0.00015
            dof_pos_limits = -0.9
            wheel_noslip = 1.0

class B2WFlatCfg( B2WRoughCfg ):
    class env:
        num_envs = 4096
        num_observations = 56
        num_privileged_obs = None
        num_actions = 16
        env_spacing = 3.
        send_timeouts = True
        episode_length_s = 20

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False
        curriculum = False
        horizontal_scale = 0.2
        vertical_scale = 0.0
        border_size = 0.0
        slope = 0.0
        roughness = 0.0
        num_rows = 1
        num_cols = 1
        spacing = 100.0
        friction = 1.0
        restitution = 0.0
        rough_terrain_friction_range = [0.5, 1.25]
        rough_terrain_restitution_range = [0.0, 0.0]
        max_init_terrain_level = 3

class B2WCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'b2w'
        max_iterations = 1500