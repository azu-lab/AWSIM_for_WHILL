"""
Learning environment of the WHILL in AWSIM (model-agnostic). Layers as in Fig. 2 of the paper:

    data_manager.py   Data management code: ROS 2 to/from AWSIM, reset of the scenario
    env.py            Learning environment management code: observation, reward, end of episode,
                      action -> Twist (Gymnasium environment WhillEnv)
    task.py           Task file (rl/configs/tasks/*.yaml): start pose, goal, limits, reward weights

Models are not here: they are in rl/agents/.

    from whill_env import make_env
    env = make_env("rl/configs/tasks/shinjuku_box.yaml")
    obs, info = env.reset()
    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
"""

from .task import Episode, Pose2D, Task, load_task  # noqa: F401  (no ROS 2 needed)


def make_env(task, seed=None):
    """task: path of a task YAML or a Task. ROS 2 is started at the first reset()."""
    from .env import WhillEnv
    if not isinstance(task, Task):
        task = load_task(task)
    return WhillEnv(task, seed=seed)
