import gym
from gym.envs.registration import register

def register(id, entry_point, force=True):
    env_specs = gym.envs.registry.env_specs
    if id in env_specs.keys():
        if not force:
            return
        del env_specs[id]
    gym.register(
        id=id,
        entry_point=entry_point,
    )

# Gazebo
# ----------------------------------------
# CartPole
register(
    id='CartPole-v0',
    entry_point='gym_gazebo2.envs.cartpole:GazeboCartPoleEnv',
)


# MARA
register(
    id='MARA-v0',
    entry_point='gym_gazebo2.envs.MARA:MARAEnv',
)

register(
    id='MARAReal-v0',
    entry_point='gym_gazebo2.envs.MARA:MARARealEnv',
)

register(
    id='MARACamera-v0',
    entry_point='gym_gazebo2.envs.MARA:MARACameraEnv',
)

register(
    id='MARAOrient-v0',
    entry_point='gym_gazebo2.envs.MARA:MARAOrientEnv',
)

register(
    id='MARACollision-v0',
    entry_point='gym_gazebo2.envs.MARA:MARACollisionEnv',
)

register(
    id='MARACollisionOrient-v0',
    entry_point='gym_gazebo2.envs.MARA:MARACollisionOrientEnv',
)

register(
    id='MARARandomTarget-v0',
    entry_point='gym_gazebo2.envs.MARA:MARARandomTargetEnv',
)
