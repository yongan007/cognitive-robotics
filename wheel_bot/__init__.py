from gym.envs.registration import register

register(
    id='wheelbot-v0',
    entry_point='wheel_bot.envs:WheelbotEnv',
)